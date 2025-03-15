/**
 * @file ld2450_parser.c
 * @brief Implementation of the HLK-LD2450 data frame parser
 * 
 * This file implements the parsing functions for the HLK-LD2450 radar data frames,
 * extracting target information and calculating derived metrics.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "ld2450.h"
#include "ld2450_private.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = LD2450_LOG_TAG;

/**
 * @brief Parse target data from a data segment within a frame
 * 
 * @param target_data Pointer to target data segment (8 bytes)
 * @param target Pointer to target structure to fill
 * @return true if target is valid, false if target segment is empty
 */
static bool parse_target(const uint8_t *target_data, ld2450_target_t *target)
{
    // Check if target segment is empty (all zeros)
    bool is_empty = true;
    for (int i = 0; i < 8; i++) {
        if (target_data[i] != 0) {
            is_empty = false;
            break;
        }
    }
    
    if (is_empty) {
        target->valid = false;
        return false;
    }
    
    // Parse target data according to protocol
    const int16_t num = 1 << 15; // 32768
    
    // X coordinate (little-endian)
    uint16_t x_raw = target_data[0] | (target_data[1] << 8);
    // Y coordinate (little-endian)
    uint16_t y_raw = target_data[2] | (target_data[3] << 8);
    // Speed (little-endian)
    uint16_t speed_raw = target_data[4] | (target_data[5] << 8);
    // Distance resolution (little-endian)
    uint16_t dist_res_raw = target_data[6] | (target_data[7] << 8);
    
    // Convert coordinates according to protocol
    // For X: if MSB is 1, value = raw - 32768; if MSB is 0, value = -raw
    target->x = (target_data[1] & 0x80) ? (x_raw - num) : -x_raw;
    // For Y: value = raw - 32768 (always subtract)
    target->y = y_raw - num;
    // For speed: if MSB is 1, value = raw - 32768; if MSB is 0, value = -raw
    target->speed = (target_data[5] & 0x80) ? (speed_raw - num) : -speed_raw;
    // Distance resolution is used directly
    target->resolution = dist_res_raw;
    
    // Calculate derived values
    target->distance = sqrt(target->x * target->x + target->y * target->y);
    target->angle = -atan2((float)target->x, (float)target->y) * (180.0f / M_PI);
    target->valid = true;
    
    return true;
}

/**
 * @brief Parse a complete data frame into a frame structure
 * 
 * @param data Raw frame data
 * @param len Length of the data
 * @param frame Frame structure to fill
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_parse_frame(const uint8_t *data, size_t len, ld2450_frame_t *frame)
{
    if (!data || !frame || len < LD2450_DATA_FRAME_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate frame header and footer
    if (memcmp(data, LD2450_DATA_FRAME_HEADER, 4) != 0 || 
        memcmp(data + len - 2, LD2450_DATA_FRAME_FOOTER, 2) != 0) {
        ESP_LOGW(TAG, "Invalid frame header or footer");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_DEBUG);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset target count
    frame->count = 0;
    
    // Get current timestamp
    frame->timestamp = esp_timer_get_time();
    
    // Parse up to 3 targets
    for (int i = 0; i < 3; i++) {
        // Target data starts at offset 4 and each target data segment is 8 bytes
        const uint8_t *target_data = data + 4 + (i * 8);
        
        // Parse target data
        if (parse_target(target_data, &frame->targets[i])) {
            frame->count++;
        } else {
            // Clear the target data for invalid targets
            memset(&frame->targets[i], 0, sizeof(ld2450_target_t));
        }
    }
    
    ESP_LOGD(TAG, "Parsed frame with %d targets", frame->count);
    
    return ESP_OK;
}

/**
 * @brief Handle a complete data frame
 * 
 * This function processes a complete data frame, parses it, and delivers
 * the results to the registered callback if any.
 * 
 * @param data Frame data
 * @param len Frame length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_handle_data_frame(const uint8_t *data, size_t len)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Skip processing if we're in config mode
    if (instance->in_config_mode) {
        return ESP_OK;
    }
    
    // Static frame structure for parsing
    static ld2450_frame_t frame;
    
    // Parse the frame
    ret = ld2450_parse_frame(data, len, &frame);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Call the callback if registered
    if (instance->target_callback != NULL) {
        instance->target_callback(&frame, instance->user_ctx);
    }
    
    return ESP_OK;
}

/**
 * @brief Process radar data coming in from UART
 * 
 * This function processes incoming data from the UART, looking for valid frame
 * headers and footers, and passing complete frames to the parser.
 * 
 * @param byte Incoming byte to process
 * @return esp_err_t ESP_OK if a frame was successfully processed, ESP_ERR_NOT_FINISHED if still collecting data, or error code
 */
esp_err_t ld2450_process_byte(uint8_t byte)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Skip data processing if we're in config mode
    if (instance->in_config_mode) {
        return ESP_ERR_NOT_FINISHED;
    }
    
    // Check for frame header
    if (!instance->frame_synced) {
        // Shift bytes through a 4-byte window to look for header
        memmove(instance->frame_buffer, instance->frame_buffer + 1, 3);
        instance->frame_buffer[3] = byte;
        
        // Check if we've found a header
        if (memcmp(instance->frame_buffer, LD2450_DATA_FRAME_HEADER, 4) == 0) {
            instance->frame_synced = true;
            instance->frame_idx = 4; // We already have 4 bytes
            ESP_LOGV(TAG, "Frame sync acquired");
        }
        
        return ESP_ERR_NOT_FINISHED;
    }
    
    // If we're collecting a frame, add the byte
    if (instance->frame_idx < LD2450_DATA_FRAME_SIZE) {
        instance->frame_buffer[instance->frame_idx++] = byte;
        
        // Check if we have a complete frame
        if (instance->frame_idx == LD2450_DATA_FRAME_SIZE) {
            // Validate frame footer
            if (memcmp(instance->frame_buffer + LD2450_DATA_FRAME_SIZE - 2, 
                      LD2450_DATA_FRAME_FOOTER, 2) == 0) {
                
                // Process the complete frame
                esp_err_t ret = ld2450_handle_data_frame(instance->frame_buffer, LD2450_DATA_FRAME_SIZE);
                
                // Reset for next frame
                instance->frame_synced = false;
                instance->frame_idx = 0;
                
                return ret;
            } else {
                // Invalid footer, resync
                ESP_LOGW(TAG, "Invalid frame footer, resyncing");
                instance->frame_synced = false;
                instance->frame_idx = 0;
                
                return ESP_ERR_INVALID_STATE;
            }
        }
    }
    
    return ESP_ERR_NOT_FINISHED;
}

/**
 * @brief Process a chunk of UART data
 * 
 * @param data UART data buffer
 * @param len Length of data
 * @return esp_err_t ESP_OK if at least one frame was processed, otherwise error code
 */
esp_err_t ld2450_process_data(const uint8_t *data, size_t len)
{
    esp_err_t ret = ESP_ERR_NOT_FINISHED;
    esp_err_t last_result = ESP_ERR_NOT_FINISHED;
    
    // Process each byte
    for (size_t i = 0; i < len; i++) {
        last_result = ld2450_process_byte(data[i]);
        if (last_result == ESP_OK) {
            ret = ESP_OK; // At least one frame was successfully processed
        }
    }
    
    return ret;
}

/**
 * @brief UART event handler
 * 
 * This function is called when UART events occur and processes incoming data.
 * 
 * @param arg UART event data
 */
void ld2450_uart_event_handler(void *arg)
{
    ld2450_state_t *instance = ld2450_get_instance();
    uart_event_t event;
    
    if (!instance || !instance->initialized) {
        return;
    }
    
    static uint8_t data_buffer[LD2450_UART_RX_BUF_SIZE];
    
    // Process UART events
    if (xQueueReceive(instance->uart_queue, &event, 0)) {
        switch (event.type) {
            case UART_DATA:
            {
                // Read data from UART
                int len = uart_read_bytes(instance->uart_port, data_buffer, 
                                         MIN(event.size, LD2450_UART_RX_BUF_SIZE),
                                         pdMS_TO_TICKS(100));
                
                if (len > 0) {
                    // Process the received data
                    ld2450_process_data(data_buffer, len);
                }
                break;
            }
            
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART FIFO overflow, clearing FIFO");
                uart_flush(instance->uart_port);
                break;
                
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART buffer full, clearing buffer");
                uart_flush(instance->uart_port);
                break;
                
            case UART_BREAK:
                ESP_LOGW(TAG, "UART break detected");
                break;
                
            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "UART parity error");
                break;
                
            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "UART frame error");
                break;
                
            default:
                ESP_LOGW(TAG, "UART event %d", event.type);
                break;
        }
    }
}

/**
 * @brief Processing task for radar data
 * 
 * This task continuously processes UART events to handle incoming radar data.
 * 
 * @param arg Task argument (not used)
 */
void ld2450_processing_task(void *arg)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 processing task started");
    
    while (instance->initialized) {
        // Process UART events
        ld2450_uart_event_handler(NULL);
        
        // Short delay to yield CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    ESP_LOGI(TAG, "LD2450 processing task stopped");
    vTaskDelete(NULL);
}