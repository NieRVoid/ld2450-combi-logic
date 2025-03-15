/**
 * @file ld2450.h
 * @brief HLK-LD2450 1T2R 24G Multi-Target Status Detection Radar driver component
 * 
 * This component provides an ESP-IDF driver for the HLK-LD2450 radar sensor, 
 * implementing the official communication protocol with emphasis on static memory 
 * allocation and efficient data handling.
 * 
 * @note This driver is compatible with ESP-IDF v5.4 and later.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Target tracking mode options
 */
typedef enum {
    LD2450_MODE_SINGLE_TARGET = 0x0001, /*!< Track a single target */
    LD2450_MODE_MULTI_TARGET = 0x0002   /*!< Track multiple targets (default) */
} ld2450_tracking_mode_t;

/**
 * @brief Serial port baud rate options
 */
typedef enum {
    LD2450_BAUD_9600   = 0x0001, /*!< 9600 baud */
    LD2450_BAUD_19200  = 0x0002, /*!< 19200 baud */
    LD2450_BAUD_38400  = 0x0003, /*!< 38400 baud */
    LD2450_BAUD_57600  = 0x0004, /*!< 57600 baud */
    LD2450_BAUD_115200 = 0x0005, /*!< 115200 baud */
    LD2450_BAUD_230400 = 0x0006, /*!< 230400 baud */
    LD2450_BAUD_256000 = 0x0007, /*!< 256000 baud (default) */
    LD2450_BAUD_460800 = 0x0008  /*!< 460800 baud */
} ld2450_baud_rate_t;

/**
 * @brief Region filtering type options
 */
typedef enum {
    LD2450_FILTER_DISABLED = 0x0000,   /*!< Disable region filtering */
    LD2450_FILTER_INCLUDE_ONLY = 0x0001, /*!< Only detect targets within specified regions */
    LD2450_FILTER_EXCLUDE = 0x0002      /*!< Do not detect targets within specified regions */
} ld2450_filter_type_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint16_t main_version;     /*!< Main version number */
    uint32_t sub_version;      /*!< Sub-version number */
    char version_string[32];   /*!< Formatted version string (e.g., "V1.02.22062416") */
} ld2450_firmware_version_t;

/**
 * @brief Region definition for filtering (rectangular area)
 */
typedef struct {
    int16_t x1;   /*!< X coordinate of first corner (mm) */
    int16_t y1;   /*!< Y coordinate of first corner (mm) */
    int16_t x2;   /*!< X coordinate of diagonal corner (mm) */
    int16_t y2;   /*!< Y coordinate of diagonal corner (mm) */
} ld2450_region_t;

/**
 * @brief Target information structure
 */
typedef struct {
    int16_t x;                /*!< X coordinate (mm) */
    int16_t y;                /*!< Y coordinate (mm) */
    int16_t speed;            /*!< Speed (cm/s) */
    uint16_t resolution;      /*!< Distance resolution (mm) */
    float distance;           /*!< Calculated distance (mm) */
    float angle;              /*!< Calculated angle in degrees */
    bool valid;               /*!< Target validity flag */
} ld2450_target_t;

/**
 * @brief Data frame structure containing target information
 */
typedef struct {
    ld2450_target_t targets[3];  /*!< Data for up to 3 targets */
    uint8_t count;               /*!< Number of valid targets (0-3) */
    int64_t timestamp;           /*!< ESP timestamp when data was received */
} ld2450_frame_t;

/**
 * @brief Driver configuration structure
 */
typedef struct {
    uart_port_t uart_port;      /*!< UART port number */
    int uart_rx_pin;            /*!< GPIO pin for UART RX */
    int uart_tx_pin;            /*!< GPIO pin for UART TX */
    uint32_t uart_baud_rate;    /*!< UART baud rate */
    bool auto_processing;       /*!< Enable automatic frame processing */
    int task_priority;          /*!< Priority for auto processing task (if enabled) */
} ld2450_config_t;

/**
 * @brief Target data callback function type
 * 
 * This function is called when new target data is available
 * 
 * @param frame Pointer to the frame containing target data
 * @param user_ctx User context pointer passed during registration
 */
typedef void (*ld2450_target_cb_t)(const ld2450_frame_t *frame, void *user_ctx);

/**
 * @brief Default configuration for the LD2450 driver
 */
#define LD2450_DEFAULT_CONFIG() { \
    .uart_port = UART_NUM_2, \
    .uart_rx_pin = 16, \
    .uart_tx_pin = 17, \
    .uart_baud_rate = 256000, \
    .auto_processing = true, \
    .task_priority = 5, \
}

/**
 * @brief Initialize the LD2450 radar driver
 * 
 * @param config Pointer to driver configuration structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_init(const ld2450_config_t *config);

/**
 * @brief Deinitialize the LD2450 radar driver and release resources
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_deinit(void);

/**
 * @brief Register a callback function for target data
 * 
 * @param callback Function pointer to call when new target data is available
 * @param user_ctx User context pointer passed to the callback function
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_register_target_callback(ld2450_target_cb_t callback, void *user_ctx);

/**
 * @brief Process a radar data frame manually
 * 
 * This function allows processing a raw data frame without using the automatic
 * processing feature. Useful for custom data acquisition.
 * 
 * @param data Raw frame data buffer
 * @param length Length of the data buffer in bytes
 * @param frame Pointer to frame structure to store parsed results
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_process_frame(const uint8_t *data, size_t length, ld2450_frame_t *frame);

/**
 * @brief Set target tracking mode (single or multi-target)
 * 
 * @param mode Tracking mode to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_tracking_mode(ld2450_tracking_mode_t mode);

/**
 * @brief Get current target tracking mode
 * 
 * @param mode Pointer to store the current tracking mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_tracking_mode(ld2450_tracking_mode_t *mode);

/**
 * @brief Get firmware version information
 * 
 * @param version Pointer to structure to store version information
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version);

/**
 * @brief Set serial port baud rate
 * 
 * This setting is saved and takes effect after module restart
 * 
 * @param baud_rate Baud rate to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate);

/**
 * @brief Restore factory default settings
 * 
 * This setting takes effect after module restart
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_restore_factory_settings(void);

/**
 * @brief Restart the radar module
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_restart_module(void);

/**
 * @brief Enable or disable Bluetooth functionality
 * 
 * This setting is persistent after power-off and takes effect after restart
 * 
 * @param enable true to enable Bluetooth, false to disable
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_bluetooth(bool enable);

/**
 * @brief Get the module's MAC address
 * 
 * @param mac Buffer to store the 6-byte MAC address
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_mac_address(uint8_t mac[6]);

/**
 * @brief Configure region filtering
 * 
 * @param type Filtering type (disabled, include only, exclude)
 * @param regions Array of 3 region definitions
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_region_filter(ld2450_filter_type_t type, const ld2450_region_t regions[3]);

/**
 * @brief Query current region filtering configuration
 * 
 * @param type Pointer to store the filtering type
 * @param regions Array of 3 region definitions to store the current configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_region_filter(ld2450_filter_type_t *type, ld2450_region_t regions[3]);

#ifdef __cplusplus
}
#endif