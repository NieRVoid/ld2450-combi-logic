/**
 * @file main.c
 * @brief Occupancy management system with multiple data sources
 *
 * This application combines occupancy data from:
 * - LD2450 radar people counter (RELIABILITY_LOW)
 * - Simulated remote control inputs (RELIABILITY_MEDIUM)
 * - Physical BOOT button on GPIO0 (RELIABILITY_HIGH)
 *
 * @author NieRVoid
 * @date 2025-03-15
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "ld2450.h"
#include "people_counter.h"
#include "occupancy_manager.h"

#define BOOT_BUTTON_GPIO 0

static const char *TAG = "OCCUPANCY_TEST";

// Source IDs for different occupancy data sources
static occupancy_source_id_t radar_source_id = 0;
static occupancy_source_id_t remote_source_id = 0;
static occupancy_source_id_t button_source_id = 0;

// Event queue for button presses to avoid processing in ISR
static QueueHandle_t button_evt_queue = NULL;

// Button debounce timer
static int64_t last_button_time = 0;

/**
 * @brief Callback for LD2450 raw target data
 */
static void ld2450_target_cb(const ld2450_frame_t *frame, void *user_ctx) {
    ESP_LOGI(TAG, "[RADAR RAW] Frame received, %d targets", frame->count);
    
    for (int i = 0; i < frame->count; i++) {
        const ld2450_target_t *target = &frame->targets[i];
        ESP_LOGI(TAG, "[RADAR RAW] Target %d: X=%d mm, Y=%d mm, Speed=%d cm/s, Distance=%.1f mm, Angle=%.1f°",
                 i + 1, target->x, target->y, target->speed, target->distance, target->angle);
    }
}

/**
 * @brief Callback for occupancy status changes
 */
static void occupancy_status_changed_cb(const occupancy_status_t *status, void *user_ctx) {
    const char *source_names[] = {"Radar", "Remote", "Button"};
    const char *source_name = "Unknown";
    
    if (status->source < 3) {
        source_name = source_names[status->source];
    }
    
    const char *reliability_str = "Unknown";
    switch (status->determining_reliability) {
        case OCCUPANCY_RELIABILITY_LOW:     reliability_str = "Low"; break;
        case OCCUPANCY_RELIABILITY_MEDIUM:  reliability_str = "Medium"; break;
        case OCCUPANCY_RELIABILITY_HIGH:    reliability_str = "High"; break;
        case OCCUPANCY_RELIABILITY_ABSOLUTE: reliability_str = "Absolute"; break;
    }
    
    ESP_LOGI(TAG, "■■■ OCCUPANCY CHANGED ■■■");
    ESP_LOGI(TAG, "Room is %s", status->is_occupied ? "OCCUPIED" : "EMPTY");
    ESP_LOGI(TAG, "Count: %d (certainty: %s)", status->count, 
             status->is_count_certain ? "Certain" : "Uncertain");
    ESP_LOGI(TAG, "Determined by: %s (Reliability: %s)", source_name, reliability_str);
    ESP_LOGI(TAG, "Power would be: %s", status->is_occupied ? "ON" : "OFF");
    ESP_LOGI(TAG, "■■■■■■■■■■■■■■■■■■■■■■■");
}

/**
 * @brief Callback for people counter updates
 */
static void people_count_changed_cb(int count, int entries, int exits, void *ctx) {
    ESP_LOGI(TAG, "[PEOPLE COUNTER] Count changed: %d (entries: %d, exits: %d)", 
             count, entries, exits);
    
    // Update occupancy manager with new count from radar
    esp_err_t ret = occupancy_manager_update_count(radar_source_id, count, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[PEOPLE COUNTER] Failed to update occupancy: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[PEOPLE COUNTER] Updated occupancy manager with count: %d", count);
    }
}

/**
 * @brief Button press ISR
 */
static void IRAM_ATTR button_isr_handler(void *arg) {
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    // Simple debounce - ignore presses within 300ms of last one
    if (current_time - last_button_time > 300) {
        last_button_time = current_time;
        uint32_t gpio_num = (uint32_t)arg;
        xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
    }
}

/**
 * @brief Task to handle button presses
 */
static void button_task(void *arg) {
    uint32_t io_num;
    
    while (1) {
        if (xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "[BUTTON] Boot button pressed - Room is OCCUPIED");
            
            // Direct update with high reliability input
            esp_err_t ret = occupancy_manager_update_count(button_source_id, 1, true);
            
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "[BUTTON] Failed to update occupancy: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "[BUTTON] Updated occupancy manager: Room is OCCUPIED");
            }
        }
    }
}

/**
 * @brief Task to simulate remote control inputs
 */
static void remote_control_task(void *arg) {
    int remote_state = 0; // Start with "no one in room" state
    
    while (1) {
        // Wait random time between 10-20 seconds
        int delay_sec = 10 + (esp_random() % 11);
        vTaskDelay(pdMS_TO_TICKS(delay_sec * 1000));
        
        // Toggle between occupied (1 person) and unoccupied (0 people)
        remote_state = !remote_state;
        
        ESP_LOGI(TAG, "[REMOTE] Remote control event: %s the room", 
                 remote_state ? "Entering" : "Exiting");
        
        // Update occupancy manager with medium reliability input
        esp_err_t ret = occupancy_manager_update_count(
            remote_source_id, remote_state, true);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "[REMOTE] Failed to update occupancy: %s", 
                     esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "[REMOTE] Updated occupancy manager with count: %d", remote_state);
        }
    }
}

/**
 * @brief Task for periodic diagnostics output
 */
static void diagnostics_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Every 5 seconds
        
        ESP_LOGI(TAG, "===== DIAGNOSTIC INFO =====");
        
        // Get current occupancy status
        occupancy_status_t status;
        esp_err_t ret = occupancy_manager_get_status(&status);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[OCCUPANCY] Room status: %s", 
                     status.is_occupied ? "OCCUPIED" : "EMPTY");
            ESP_LOGI(TAG, "[OCCUPANCY] Count: %d (by %s, certainty: %s)",
                     status.count, 
                     status.source == radar_source_id ? "Radar" : 
                     status.source == remote_source_id ? "Remote" : 
                     status.source == button_source_id ? "Button" : "Unknown",
                     status.is_count_certain ? "Certain" : "Uncertain");
        }
        
        // Get radar people counter details
        int count, entries, exits;
        ret = people_counter_get_details(&count, &entries, &exits);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[PEOPLE COUNTER] Count=%d, Entries=%d, Exits=%d", 
                     count, entries, exits);
        }
        
        ESP_LOGI(TAG, "==========================");
    }
}

void app_main(void) {
    esp_err_t ret;
    
    // Print header
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Occupancy Management System Test");
    ESP_LOGI(TAG, "====================================");
    
    // Initialize button input
    button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, (void*)BOOT_BUTTON_GPIO);
    
    ESP_LOGI(TAG, "Button input initialized (GPIO0)");
    
    // Initialize occupancy manager
    ESP_ERROR_CHECK(occupancy_manager_init());
    ESP_LOGI(TAG, "Occupancy manager initialized");
    
    // Register occupancy sources
    ESP_ERROR_CHECK(occupancy_manager_register_source(
        "Radar", OCCUPANCY_RELIABILITY_LOW, OCCUPANCY_SOURCE_CONTINUOUS, 
        5000, -1, &radar_source_id));
    
    ESP_ERROR_CHECK(occupancy_manager_register_source(
        "Remote", OCCUPANCY_RELIABILITY_MEDIUM, OCCUPANCY_SOURCE_TRIGGERED, 
        30000, -1, &remote_source_id));
    
    ESP_ERROR_CHECK(occupancy_manager_register_source(
        "Button", OCCUPANCY_RELIABILITY_HIGH, OCCUPANCY_SOURCE_TRIGGERED, 
        0, 0, &button_source_id));
    
    ESP_LOGI(TAG, "Registered sources: Radar(ID=%d), Remote(ID=%d), Button(ID=%d)", 
             radar_source_id, remote_source_id, button_source_id);
    
    // Register callback for occupancy changes
    ESP_ERROR_CHECK(occupancy_manager_register_callback(
        occupancy_status_changed_cb, NULL));
    
    // Initialize LD2450 radar driver
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    radar_config.uart_port = UART_NUM_2;
    radar_config.uart_rx_pin = 16; 
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    
    ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize radar: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register callback for radar target data
    ret = ld2450_register_target_callback(ld2450_target_cb, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register radar callback: %s", esp_err_to_name(ret));
        return;
    }
    
    // Display firmware and configuration info once at initialization
    ld2450_firmware_version_t version;
    ret = ld2450_get_firmware_version(&version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LD2450 Firmware: %s", version.version_string);
    }
    
    ld2450_tracking_mode_t mode;
    ret = ld2450_get_tracking_mode(&mode);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LD2450 Tracking mode: %s", 
                 mode == LD2450_MODE_SINGLE_TARGET ? "Single target" : "Multi-target");
    }
    
    ESP_LOGI(TAG, "LD2450 radar driver initialized");
    
    // Initialize people counter
    people_counter_config_t pc_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    pc_config.vector_threshold = 1200;          // 120cm movement to count entry/exit
    pc_config.empty_target_threshold = 5;       // 5 empty frames to consider target gone
    pc_config.detection_min_x = -2000;          // 4m wide detection area
    pc_config.detection_max_x = 2000;
    pc_config.detection_min_y = 0;              // 3m deep detection area
    pc_config.detection_max_y = 3000;
    pc_config.count_changed_cb = people_count_changed_cb;
    
    ret = people_counter_init(&pc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize people counter: %s", esp_err_to_name(ret));
        ld2450_deinit();
        return;
    }
    ESP_LOGI(TAG, "People counter initialized");
    
    // Configure region filtering for optimal people counting
    ret = people_counter_configure_region();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure radar regions: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Radar regions configured for people counting");
    }
    
    // Create tasks
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(remote_control_task, "remote_task", 2048, NULL, 5, NULL);
    xTaskCreate(diagnostics_task, "diag_task", 3072, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "System initialized and running");
    ESP_LOGI(TAG, "- Button presses simulate physical presence (room occupied)");
    ESP_LOGI(TAG, "- Remote control simulated every 10-20 seconds");
    ESP_LOGI(TAG, "- Radar detection running continuously");
    ESP_LOGI(TAG, "- Diagnostics displayed every 5 seconds");
}