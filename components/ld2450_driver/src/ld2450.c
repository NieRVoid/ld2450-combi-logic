/**
 * @file ld2450.c
 * @brief Main implementation of the HLK-LD2450 driver
 * 
 * This file implements the main driver functionality for the HLK-LD2450 radar sensor,
 * including initialization, deinitialization, and core operations.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ld2450.h"
#include "ld2450_private.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_timer.h"

static const char *TAG = LD2450_LOG_TAG;

// Global driver instance (singleton)
static ld2450_state_t s_ld2450_state = {0};

/**
 * @brief Get driver instance
 * 
 * @return Pointer to driver state structure
 */
ld2450_state_t *ld2450_get_instance(void)
{
    return &s_ld2450_state;
}

/**
 * @brief Initialize the LD2450 radar driver
 * 
 * @param config Pointer to driver configuration structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_init(const ld2450_config_t *config)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret = ESP_OK;
    
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (instance->initialized) {
        ESP_LOGW(TAG, "LD2450 driver already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Zero out the state structure
    memset(instance, 0, sizeof(ld2450_state_t));
    
    // Store configuration
    instance->uart_port = config->uart_port;
    instance->rx_pin = config->uart_rx_pin;
    instance->tx_pin = config->uart_tx_pin;
    instance->baud_rate = config->uart_baud_rate;
    instance->auto_processing = config->auto_processing;
    
    // Create mutex for thread safety
    instance->mutex = xSemaphoreCreateMutex();
    if (!instance->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ret = uart_driver_install(config->uart_port, LD2450_UART_RX_BUF_SIZE * 2, 
                              0, 20, &instance->uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Configure UART parameters
    ret = uart_param_config(config->uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Set UART pins
    ret = uart_set_pin(config->uart_port, config->uart_tx_pin, config->uart_rx_pin, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Configure GPIO pull-up for reliability
    gpio_set_pull_mode(config->uart_rx_pin, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(config->uart_tx_pin, GPIO_PULLUP_ONLY);
    
    // Initialize frame buffer index and sync state
    instance->frame_idx = 0;
    instance->frame_synced = false;
    instance->in_config_mode = false;
    
    // Set initialized flag
    instance->initialized = true;
    
    ESP_LOGI(TAG, "LD2450 driver initialized on UART%" PRIu32 " (RX: GPIO%" PRIu32 ", TX: GPIO%" PRIu32 ", baud: %" PRIu32 ")",  
        (uint32_t)instance->uart_port, (uint32_t)instance->rx_pin, (uint32_t)instance->tx_pin, instance->baud_rate);
    
    // Start processing task if auto-processing is enabled
    if (config->auto_processing) {
        xTaskCreate(ld2450_processing_task, "ld2450_task", LD2450_TASK_STACK_SIZE,
                    NULL, config->task_priority, &instance->task_handle);
                    
        if (!instance->task_handle) {
            ESP_LOGE(TAG, "Failed to create processing task");
            ld2450_deinit();
            return ESP_ERR_NO_MEM;
        }
        
        ESP_LOGI(TAG, "Auto-processing enabled with task priority %d", config->task_priority);
    }
    
    return ESP_OK;
}

/**
 * @brief Deinitialize the LD2450 radar driver and release resources
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_deinit(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Make sure we exit configuration mode if active
    if (instance->in_config_mode) {
        ld2450_exit_config_mode();
    }
    
    // Delete task if it was created
    if (instance->task_handle) {
        instance->initialized = false;  // Signal task to exit
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
        
        // If task still running, delete it
        if (eTaskGetState(instance->task_handle) != eDeleted) {
            vTaskDelete(instance->task_handle);
        }
        
        instance->task_handle = NULL;
    }
    
    // Delete UART driver
    uart_driver_delete(instance->uart_port);
    
    // Delete mutex
    if (instance->mutex) {
        vSemaphoreDelete(instance->mutex);
        instance->mutex = NULL;
    }
    
    // Reset state
    memset(instance, 0, sizeof(ld2450_state_t));
    
    ESP_LOGI(TAG, "LD2450 driver deinitialized");
    
    return ESP_OK;
}

/**
 * @brief Register a callback function for target data
 * 
 * @param callback Function pointer to call when new target data is available
 * @param user_ctx User context pointer passed to the callback function
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_register_target_callback(ld2450_target_cb_t callback, void *user_ctx)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        instance->target_callback = callback;
        instance->user_ctx = user_ctx;
        xSemaphoreGive(instance->mutex);
        
        ESP_LOGI(TAG, "Target callback %sregistered", callback ? "" : "un");
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Process a radar data frame manually
 * 
 * @param data Raw frame data buffer
 * @param length Length of the data buffer in bytes
 * @param frame Pointer to frame structure to store parsed results
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_process_frame(const uint8_t *data, size_t length, ld2450_frame_t *frame)
{
    if (!data || !frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (length != LD2450_DATA_FRAME_SIZE) {
        ESP_LOGW(TAG, "Invalid frame size: %zu bytes (expected %d)", length, LD2450_DATA_FRAME_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ld2450_parse_frame(data, length, frame);
}

/* 
 * The following functions are implemented in ld2450_config.c and are already declared
 * in ld2450.h. We don't need to re-implement them here, as they are accessible through
 * the public API header.
 * 
 * - ld2450_set_tracking_mode
 * - ld2450_get_tracking_mode
 * - ld2450_get_firmware_version
 * - ld2450_set_baud_rate
 * - ld2450_restore_factory_settings
 * - ld2450_restart_module
 * - ld2450_set_bluetooth
 * - ld2450_get_mac_address
 * - ld2450_set_region_filter
 * - ld2450_get_region_filter
 */