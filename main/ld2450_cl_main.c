/**
 * @file main.c
 * @brief Multi-source occupancy detection system using LD2450 radar
 *
 * This application integrates three components:
 * - ld2450_driver: Radar sensor driver
 * - people_counter: People counting logic
 * - occupancy_manager: Multiple source occupancy determination
 *
 * Three occupancy sources are implemented:
 * 1. Radar-based people counting (high accuracy, continuous)
 * 2. BOOT button on GPIO0 (high reliability, triggered)
 * 3. Remote control placeholder (medium reliability, triggered)
 *
 * @author NieRVoid
 * @date 2025-03-18
 * @license MIT
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

// Include components' headers
#include "ld2450.h"
#include "people_counter.h"
#include "occupancy_manager.h"

static const char *TAG = "occupancy-app";

// Source IDs for occupancy manager
static occupancy_source_id_t radar_source_id;
static occupancy_source_id_t button_source_id;
static occupancy_source_id_t remote_source_id;

// Queue for button events
static QueueHandle_t button_event_queue;

// Simulated remote control command
typedef enum {
    REMOTE_CMD_NONE = 0,
    REMOTE_CMD_OCCUPIED,
    REMOTE_CMD_VACANT
} remote_cmd_t;

// Function prototypes
static void button_task(void *arg);
static void occupancy_status_callback(const occupancy_status_t *status, void *user_ctx);
static void people_counter_callback(int count, int entries, int exits, void *context);
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx);
static void remote_control_task(void *arg);
static void button_isr_handler(void *arg);
static void print_occupancy_status(const occupancy_status_t *status);

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting occupancy detection system...");
    
    // Initialize queue for button events
    button_event_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!button_event_queue) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return;
    }
    
    // ------------------------------------------------------------------------
    // 1. Initialize LD2450 radar driver
    // ------------------------------------------------------------------------
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    // radar_config.uart_port = UART_NUM_2;
    radar_config.uart_rx_pin = 16;
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    // radar_config.auto_processing = true;
    
    ret = ld2450_init(&radar_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450 radar: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register radar data callback
    ret = ld2450_register_target_callback(radar_target_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register radar callback: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 radar initialized successfully");
    
    // ------------------------------------------------------------------------
    // 2. Initialize people counter
    // ------------------------------------------------------------------------
    people_counter_config_t counter_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    counter_config.vector_threshold = 1000;          // 100cm movement to count as entry/exit
    counter_config.empty_target_threshold = 5;       // 5 empty frames to consider target gone
    counter_config.detection_min_x = -2000;          // Detection area: 4m wide, 2m deep
    counter_config.detection_max_x = 2000;
    counter_config.detection_min_y = 0;
    counter_config.detection_max_y = 2000;

    counter_config.count_changed_cb = people_counter_callback;
    
    ret = people_counter_init(&counter_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize people counter: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "People counter initialized successfully");
    
    // Configure detection region
    ret = people_counter_configure_region();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure detection region: %s", esp_err_to_name(ret));
        return;
    }
    
    // ------------------------------------------------------------------------
    // 3. Initialize occupancy manager
    // ------------------------------------------------------------------------
    ret = occupancy_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize occupancy manager: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Occupancy manager initialized successfully");
    
    // Register callback for occupancy status changes
    ret = occupancy_manager_register_callback(occupancy_status_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register occupancy callback: %s", esp_err_to_name(ret));
        return;
    }
    
    // ------------------------------------------------------------------------
    // 4. Register occupancy sources
    // ------------------------------------------------------------------------
    
    // Register radar as continuous source with low reliability
    ret = occupancy_manager_register_source(
        "Radar", 
        OCCUPANCY_RELIABILITY_LOW, 
        OCCUPANCY_SOURCE_CONTINUOUS,
        5000,  // 5 second timeout
        -1,    // Initial count unknown
        &radar_source_id
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register radar source: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Radar registered as source ID: %d", radar_source_id);
    
    // Register button as high reliability triggered source
    ret = occupancy_manager_register_source(
        "Button", 
        OCCUPANCY_RELIABILITY_HIGH, 
        OCCUPANCY_SOURCE_TRIGGERED,
        30000,  // 30 second timeout
        -1,     // Initial count unknown
        &button_source_id
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register button source: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Button registered as source ID: %d", button_source_id);
    
    // Register remote as medium reliability triggered source
    ret = occupancy_manager_register_source(
        "Remote", 
        OCCUPANCY_RELIABILITY_MEDIUM, 
        OCCUPANCY_SOURCE_TRIGGERED,
        60000,  // 60 second timeout
        -1,     // Initial count unknown
        &remote_source_id
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register remote source: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Remote registered as source ID: %d", remote_source_id);
    
    // ------------------------------------------------------------------------
    // 5. Configure BOOT button (GPIO0)
    // ------------------------------------------------------------------------
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, button_isr_handler, NULL);
    ESP_LOGI(TAG, "BOOT button configured on GPIO0");
    
    // ------------------------------------------------------------------------
    // 6. Start tasks
    // ------------------------------------------------------------------------
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(remote_control_task, "remote_control_task", 2048, NULL, 10, NULL);
    
    // Get initial occupancy status
    occupancy_status_t status;
    if (occupancy_manager_get_status(&status) == ESP_OK) {
        ESP_LOGI(TAG, "Initial occupancy status:");
        print_occupancy_status(&status);
    }
    
    ESP_LOGI(TAG, "Occupancy detection system running");
}

/**
 * @brief Process button press events
 */
static void button_task(void *arg)
{
    uint32_t gpio_num;
    
    while (1) {
        if (xQueueReceive(button_event_queue, &gpio_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "BOOT button pressed (GPIO %"PRIu32")", gpio_num);
            
            // When button is pressed, trigger presence with at least 1 person
            esp_err_t ret = occupancy_manager_trigger_presence(button_source_id, 1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to trigger button presence: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "Button source triggered presence (count=1)");
            }
            
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
}

/**
 * @brief Simulate remote control commands
 * 
 * This task simulates receiving commands from a remote control
 * by alternating between occupied and vacant states every 30 seconds
 */
static void remote_control_task(void *arg)
{
    remote_cmd_t current_cmd = REMOTE_CMD_NONE;
    int simulated_count = 0;
    
    while (1) {
        // Simulate receiving a remote command every 3000 seconds
        vTaskDelay(pdMS_TO_TICKS(3000000));
        
        // Alternate between occupied and vacant
        if (current_cmd == REMOTE_CMD_NONE || current_cmd == REMOTE_CMD_VACANT) {
            current_cmd = REMOTE_CMD_OCCUPIED;
            simulated_count = 2;  // Simulate 2 people
            ESP_LOGI(TAG, "Remote control: OCCUPIED command received");
        } else {
            current_cmd = REMOTE_CMD_VACANT;
            simulated_count = 0;  // Simulate empty room
            ESP_LOGI(TAG, "Remote control: VACANT command received");
        }
        
        // Update occupancy manager with remote control data
        esp_err_t ret = occupancy_manager_update_count(remote_source_id, simulated_count, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update remote control count: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Remote source updated (count=%d)", simulated_count);
        }
    }
}

/**
 * @brief Button interrupt service routine
 */
static void button_isr_handler(void *arg)
{
    uint32_t gpio_num = GPIO_NUM_0;
    xQueueSendFromISR(button_event_queue, &gpio_num, NULL);
}

/**
 * @brief Callback for radar target data
 */
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx)
{
    // Only log when we have targets
    if (frame->count > 0) {
        ESP_LOGI(TAG, "Radar detected %d target(s):", frame->count);
        for (int i = 0; i < frame->count; i++) {
            ESP_LOGI(TAG, "  Target %d: x=%d mm, y=%d mm, speed=%d cm/s", 
                i + 1, frame->targets[i].x, frame->targets[i].y, frame->targets[i].speed);
        }
    }
}

/**
 * @brief Callback for people counter events
 */
static void people_counter_callback(int count, int entries, int exits, void *context)
{
    ESP_LOGI(TAG, "People counter: count=%d, entries=%d, exits=%d", count, entries, exits);
    
    // Update the occupancy manager with the new count from radar
    esp_err_t ret = occupancy_manager_update_count(radar_source_id, count, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update radar count: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback for occupancy status changes
 */
static void occupancy_status_callback(const occupancy_status_t *status, void *user_ctx)
{
    ESP_LOGI(TAG, "Occupancy status changed:");
    print_occupancy_status(status);
}

/**
 * @brief Print occupancy status details
 */
static void print_occupancy_status(const occupancy_status_t *status)
{
    const char *source_names[] = {
        "Radar",
        "Button",
        "Remote"
    };
    
    const char *reliability_names[] = {
        "Low",
        "Medium",
        "High",
        "Absolute"
    };
    
    // Map source ID to name (this is a simplification, actual IDs might be different)
    const char *source_name = "Unknown";
    if (status->source == radar_source_id) {
        source_name = "Radar";
    } else if (status->source == button_source_id) {
        source_name = "Button";
    } else if (status->source == remote_source_id) {
        source_name = "Remote";
    }
    
    ESP_LOGI(TAG, "  Occupancy: %s (count: %d%s)", 
        status->is_occupied ? "OCCUPIED" : "VACANT",
        status->count,
        status->is_count_certain ? ", certain" : ", uncertain");
    
    ESP_LOGI(TAG, "  Determining source: %s (reliability: %s)", 
        source_name,
        reliability_names[status->determining_reliability]);
}