/**
 * @file ld2450_config.c
 * @brief Implementation of the HLK-LD2450 configuration functions
 * 
 * This file implements the configuration command functions for the HLK-LD2450 radar sensor,
 * including entering/exiting configuration mode and sending various configuration commands.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#include <string.h>
#include "ld2450.h"
#include "ld2450_private.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = LD2450_LOG_TAG;

/**
 * @brief Build a command packet in the command buffer
 * 
 * @param instance Driver instance
 * @param cmd Command word
 * @param value Command value buffer
 * @param value_len Length of the command value in bytes
 * @return Size of the command packet
 */
static size_t build_command_packet(ld2450_state_t *instance, ld2450_cmd_t cmd, 
                                  const void *value, size_t value_len)
{
    size_t idx = 0;
    
    // Add header
    memcpy(&instance->cmd_buffer[idx], LD2450_CONFIG_FRAME_HEADER, 4);
    idx += 4;
    
    // Add data length (command word (2 bytes) + value length)
    instance->cmd_buffer[idx++] = (value_len + 2) & 0xFF;
    instance->cmd_buffer[idx++] = ((value_len + 2) >> 8) & 0xFF;
    
    // Add command word (little-endian)
    instance->cmd_buffer[idx++] = cmd & 0xFF;
    instance->cmd_buffer[idx++] = (cmd >> 8) & 0xFF;
    
    // Add command value if present
    if (value != NULL && value_len > 0) {
        memcpy(&instance->cmd_buffer[idx], value, value_len);
        idx += value_len;
    }
    
    // Add footer
    memcpy(&instance->cmd_buffer[idx], LD2450_CONFIG_FRAME_FOOTER, 4);
    idx += 4;
    
    return idx;
}

/**
 * @brief Send a command packet and wait for acknowledgement
 * 
 * @param cmd Command word
 * @param value Command value buffer
 * @param value_len Length of the command value in bytes
 * @param ack_buffer Buffer to store the ACK response
 * @param ack_len Pointer to store ACK response length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_send_command(ld2450_cmd_t cmd, const void *value, size_t value_len, 
                             uint8_t *ack_buffer, size_t *ack_len, uint32_t timeout_ms)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret = ESP_OK;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(instance->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for command %04x", cmd);
        return ESP_ERR_TIMEOUT;
    }
    
    // Build the command packet
    size_t cmd_len = build_command_packet(instance, cmd, value, value_len);
    
    // Clear UART RX buffer before sending command
    uart_flush(instance->uart_port);
    
    // Send the command
    int bytes_sent = uart_write_bytes(instance->uart_port, (const char *)instance->cmd_buffer, cmd_len);
    if (bytes_sent != cmd_len) {
        ESP_LOGE(TAG, "Failed to send command %04x (sent %d/%zu bytes)", cmd, bytes_sent, cmd_len);
        xSemaphoreGive(instance->mutex);
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Sent command %04x (%zu bytes):", cmd, cmd_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, instance->cmd_buffer, cmd_len, ESP_LOG_VERBOSE);
    
    // Wait for ACK response
    uint8_t byte_buf;
    int idx = 0;
    bool found_header = false;
    bool found_footer = false;
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while ((xTaskGetTickCount() - start_time) < timeout_ticks && idx < LD2450_ACK_BUFFER_SIZE) {
        int bytes_read = uart_read_bytes(instance->uart_port, &byte_buf, 1, 10 / portTICK_PERIOD_MS);
        if (bytes_read == 1) {
            instance->ack_buffer[idx] = byte_buf;
            idx++;
            
            // Check for header
            if (idx >= 4 && !found_header) {
                if (memcmp(instance->ack_buffer, LD2450_CONFIG_FRAME_HEADER, 4) == 0) {
                    found_header = true;
                }
            }
            
            // Check for footer once we have a header
            if (found_header && idx >= 8) {  // Header + min data size + footer
                if (memcmp(&instance->ack_buffer[idx - 4], LD2450_CONFIG_FRAME_FOOTER, 4) == 0) {
                    found_footer = true;
                    break;
                }
            }
        }
    }
    
    if (!found_header || !found_footer) {
        ESP_LOGE(TAG, "Failed to receive complete ACK for command %04x (got %d bytes)", cmd, idx);
        if (idx > 0) {
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, instance->ack_buffer, idx, ESP_LOG_DEBUG);
        }
        xSemaphoreGive(instance->mutex);
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG, "Received ACK for command %04x (%d bytes):", cmd, idx);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, instance->ack_buffer, idx, ESP_LOG_VERBOSE);
    
    if (ack_buffer != NULL && ack_len != NULL) {
        memcpy(ack_buffer, instance->ack_buffer, idx);
        *ack_len = idx;
    }
    
    // Validate ACK
    ret = ld2450_validate_ack(instance->ack_buffer, idx, cmd);
    
    xSemaphoreGive(instance->mutex);
    return ret;
}

/**
 * @brief Validate an ACK response for a specific command
 * 
 * @param ack ACK buffer
 * @param len ACK length
 * @param cmd Command to validate
 * @return esp_err_t ESP_OK if valid, error code otherwise
 */
esp_err_t ld2450_validate_ack(const uint8_t *ack, size_t len, ld2450_cmd_t cmd)
{
    // Minimum length check (header + data length + cmd + status + footer)
    if (len < 10) {
        ESP_LOGE(TAG, "ACK too short: %zu bytes", len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Validate header and footer
    if (memcmp(ack, LD2450_CONFIG_FRAME_HEADER, 4) != 0 || 
        memcmp(&ack[len - 4], LD2450_CONFIG_FRAME_FOOTER, 4) != 0) {
        ESP_LOGE(TAG, "Invalid ACK header/footer");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Check command echo
    if (ack[6] != (cmd & 0xFF) || ack[7] != 0x01) {
        ESP_LOGE(TAG, "Invalid command echo in ACK: %02x %02x", ack[6], ack[7]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Check status code
    if (ack[8] != 0x00 || ack[9] != 0x00) {
        ESP_LOGE(TAG, "Command %04x failed with status %02x %02x", cmd, ack[8], ack[9]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

/**
 * @brief Enter configuration mode
 * 
 * This function must be called before sending any other configuration commands.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_enter_config_mode(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (instance->in_config_mode) {
        ESP_LOGW(TAG, "Already in configuration mode");
        return ESP_OK;
    }
    
    // Command value: 0x0001 (little-endian)
    uint8_t value[2] = {0x01, 0x00};
    esp_err_t ret = ld2450_send_command(LD2450_CMD_ENABLE_CONFIG, value, sizeof(value), 
                                       NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        instance->in_config_mode = true;
        ESP_LOGI(TAG, "Entered configuration mode");
    } else {
        ESP_LOGE(TAG, "Failed to enter configuration mode: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Exit configuration mode
 * 
 * This function must be called after configuration is complete to return
 * the radar to normal operating mode.
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_exit_config_mode(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!instance->in_config_mode) {
        ESP_LOGW(TAG, "Not in configuration mode");
        return ESP_OK;
    }
    
    esp_err_t ret = ld2450_send_command(LD2450_CMD_END_CONFIG, NULL, 0, 
                                       NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        instance->in_config_mode = false;
        ESP_LOGI(TAG, "Exited configuration mode");
    } else {
        ESP_LOGE(TAG, "Failed to exit configuration mode: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Set target tracking mode
 * 
 * @param mode Tracking mode to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_tracking_mode(ld2450_tracking_mode_t mode)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set tracking mode
    ld2450_cmd_t cmd = (mode == LD2450_MODE_SINGLE_TARGET) ? 
                       LD2450_CMD_SINGLE_TARGET : LD2450_CMD_MULTI_TARGET;
    
    ret = ld2450_send_command(cmd, NULL, 0, NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Get current target tracking mode
 * 
 * @param mode Pointer to store the current tracking mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_tracking_mode(ld2450_tracking_mode_t *mode)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    size_t ack_len;
    uint8_t ack_buffer[LD2450_ACK_BUFFER_SIZE];
    
    if (!instance || !instance->initialized || !mode) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Query tracking mode
    ret = ld2450_send_command(LD2450_CMD_QUERY_TARGET_MODE, NULL, 0, 
                             ack_buffer, &ack_len, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK && ack_len >= 12) {
        // Extract mode value from ACK (little-endian)
        uint16_t mode_value = ack_buffer[10] | (ack_buffer[11] << 8);
        
        switch (mode_value) {
            case 0x0001:
                *mode = LD2450_MODE_SINGLE_TARGET;
                break;
            case 0x0002:
                *mode = LD2450_MODE_MULTI_TARGET;
                break;
            default:
                ESP_LOGW(TAG, "Unknown target mode: %04x", mode_value);
                ret = ESP_ERR_INVALID_RESPONSE;
                break;
        }
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Get firmware version information
 * 
 * @param version Pointer to structure to store version information
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    size_t ack_len;
    uint8_t ack_buffer[LD2450_ACK_BUFFER_SIZE];
    
    if (!instance || !instance->initialized || !version) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Query firmware version
    ret = ld2450_send_command(LD2450_CMD_READ_FW_VERSION, NULL, 0, 
                             ack_buffer, &ack_len, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK && ack_len >= 22) {  // Make sure we have enough data (header(4) + len(2) + cmd(2) + status(2) + fw_type(2) + main_ver(2) + sub_ver(4) + footer(4))
        // Extract firmware version information based on protocol documentation
        // Main version is at offset 12-13 (little-endian)
        version->main_version = ack_buffer[12] | (ack_buffer[13] << 8);
        
        // Sub-version is at offset 14-17 (little-endian)
        version->sub_version = ack_buffer[14] | 
                              (ack_buffer[15] << 8) | 
                              ((uint32_t)ack_buffer[16] << 16) | 
                              ((uint32_t)ack_buffer[17] << 24);
        
        // Format version string according to the protocol example (V1.02.22062416)
        // Main version is split: lower byte is the first number, upper byte is after the first dot
        snprintf(version->version_string, sizeof(version->version_string),
                "V%u.%02u.%08lX", 
                version->main_version & 0xFF,         // Lower byte of main version
                (version->main_version >> 8) & 0xFF,  // Upper byte of main version
                (unsigned long)version->sub_version); // Sub-version as a hex value
        
        ESP_LOGI(TAG, "Firmware version: %s", version->version_string);
    } else {
        ESP_LOGE(TAG, "Failed to read firmware version or invalid response");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Set serial port baud rate
 * 
 * @param baud_rate Baud rate to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate baud rate
    if (baud_rate < LD2450_BAUD_9600 || baud_rate > LD2450_BAUD_460800) {
        ESP_LOGE(TAG, "Invalid baud rate: %d", baud_rate);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set baud rate (little-endian)
    uint8_t value[2] = {baud_rate & 0xFF, (baud_rate >> 8) & 0xFF};
    ret = ld2450_send_command(LD2450_CMD_SET_BAUD_RATE, value, sizeof(value), 
                             NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Baud rate set to index %d", baud_rate);
    } else {
        ESP_LOGE(TAG, "Failed to set baud rate: %s", esp_err_to_name(ret));
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Restore factory default settings
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_restore_factory_settings(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Restore factory settings
    ret = ld2450_send_command(LD2450_CMD_RESTORE_FACTORY, NULL, 0, 
                             NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Factory settings restored");
    } else {
        ESP_LOGE(TAG, "Failed to restore factory settings: %s", esp_err_to_name(ret));
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Restart the radar module
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_restart_module(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Restart module
    ret = ld2450_send_command(LD2450_CMD_RESTART_MODULE, NULL, 0, 
                             NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Module restart command sent successfully");
        
        // Wait for module to restart
        vTaskDelay(pdMS_TO_TICKS(LD2450_RESTART_TIMEOUT_MS));
        
        // Reset configuration mode state since module restarted
        instance->in_config_mode = false;
    } else {
        ESP_LOGE(TAG, "Failed to restart module: %s", esp_err_to_name(ret));
        
        // Try to exit configuration mode
        ld2450_exit_config_mode();
    }
    
    return ret;
}

/**
 * @brief Enable or disable Bluetooth functionality
 * 
 * @param enable true to enable Bluetooth, false to disable
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_bluetooth(bool enable)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set Bluetooth state (little-endian)
    uint8_t value[2] = {enable ? 0x01 : 0x00, 0x00};
    ret = ld2450_send_command(LD2450_CMD_SET_BLUETOOTH, value, sizeof(value), 
                             NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth %s", enable ? "enabled" : "disabled");
    } else {
        ESP_LOGE(TAG, "Failed to set Bluetooth state: %s", esp_err_to_name(ret));
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Get the module's MAC address
 * 
 * @param mac Buffer to store the 6-byte MAC address
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_mac_address(uint8_t mac[6])
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    size_t ack_len;
    uint8_t ack_buffer[LD2450_ACK_BUFFER_SIZE];
    
    if (!instance || !instance->initialized || !mac) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Command value: 0x0001 (little-endian)
    uint8_t value[2] = {0x01, 0x00};
    ret = ld2450_send_command(LD2450_CMD_GET_MAC_ADDRESS, value, sizeof(value), 
                             ack_buffer, &ack_len, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK && ack_len >= 16) {
        // Extract MAC address (6 bytes starting at offset 10)
        memcpy(mac, &ack_buffer[10], 6);
        
        ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        ESP_LOGE(TAG, "Failed to get MAC address or invalid response");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Set region filtering configuration
 * 
 * @param type Filtering type (disabled, include only, exclude)
 * @param regions Array of 3 region definitions
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_region_filter(ld2450_filter_type_t type, const ld2450_region_t regions[3])
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    
    if (!instance || !instance->initialized || !regions) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Build command value buffer (26 bytes total)
    uint8_t value[26];
    int idx = 0;
    
    // Filter type (2 bytes, little-endian)
    value[idx++] = type & 0xFF;
    value[idx++] = (type >> 8) & 0xFF;
    
    // 3 regions (each 8 bytes: x1,y1,x2,y2 as int16_t, little-endian)
    for (int i = 0; i < 3; i++) {
        // Region corner 1
        value[idx++] = regions[i].x1 & 0xFF;
        value[idx++] = (regions[i].x1 >> 8) & 0xFF;
        value[idx++] = regions[i].y1 & 0xFF;
        value[idx++] = (regions[i].y1 >> 8) & 0xFF;
        
        // Region corner 2
        value[idx++] = regions[i].x2 & 0xFF;
        value[idx++] = (regions[i].x2 >> 8) & 0xFF;
        value[idx++] = regions[i].y2 & 0xFF;
        value[idx++] = (regions[i].y2 >> 8) & 0xFF;
    }
    
    // Send command
    ret = ld2450_send_command(LD2450_CMD_SET_REGION, value, sizeof(value), 
                             NULL, NULL, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Region filtering set to type %d", type);
    } else {
        ESP_LOGE(TAG, "Failed to set region filtering: %s", esp_err_to_name(ret));
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}

/**
 * @brief Query current region filtering configuration
 * 
 * @param type Pointer to store the filtering type
 * @param regions Array of 3 region definitions to store the current configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_region_filter(ld2450_filter_type_t *type, ld2450_region_t regions[3])
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret;
    size_t ack_len;
    uint8_t ack_buffer[LD2450_ACK_BUFFER_SIZE];
    
    if (!instance || !instance->initialized || !type || !regions) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Enter configuration mode
    ret = ld2450_enter_config_mode();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Query region filtering
    ret = ld2450_send_command(LD2450_CMD_QUERY_REGION, NULL, 0, 
                             ack_buffer, &ack_len, LD2450_CONFIG_TIMEOUT_MS);
    
    if (ret == ESP_OK && ack_len >= 40) {
        // Extract filter type (2 bytes, little-endian)
        *type = ack_buffer[10] | (ack_buffer[11] << 8);
        
        // Extract 3 regions
        for (int i = 0; i < 3; i++) {
            int offset = 12 + (i * 8);
            
            // Region corner 1
            regions[i].x1 = ack_buffer[offset] | (ack_buffer[offset + 1] << 8);
            regions[i].y1 = ack_buffer[offset + 2] | (ack_buffer[offset + 3] << 8);
            
            // Region corner 2
            regions[i].x2 = ack_buffer[offset + 4] | (ack_buffer[offset + 5] << 8);
            regions[i].y2 = ack_buffer[offset + 6] | (ack_buffer[offset + 7] << 8);
        }
        
        ESP_LOGI(TAG, "Region filtering type: %d", *type);
        
        for (int i = 0; i < 3; i++) {
            if (regions[i].x1 == 0 && regions[i].y1 == 0 && 
                regions[i].x2 == 0 && regions[i].y2 == 0) {
                ESP_LOGI(TAG, "Region %d: Not configured", i + 1);
            } else {
                ESP_LOGI(TAG, "Region %d: (%d,%d) - (%d,%d)", 
                         i + 1, regions[i].x1, regions[i].y1, regions[i].x2, regions[i].y2);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to query region filtering or invalid response");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    
    // Exit configuration mode
    esp_err_t exit_ret = ld2450_exit_config_mode();
    
    // Return the first error if any
    return (ret != ESP_OK) ? ret : exit_ret;
}