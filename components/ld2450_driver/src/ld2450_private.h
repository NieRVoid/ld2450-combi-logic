/**
 * @file ld2450_private.h
 * @brief Private header for HLK-LD2450 driver internal definitions
 * 
 * This file contains internal definitions and structures used by the LD2450 driver.
 * These declarations are not part of the public API and should not be used directly
 * by applications.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ld2450.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Log tag for the LD2450 driver */
#define LD2450_LOG_TAG "LD2450"

/** @brief Size of the ACK buffer for configuration commands */
#define LD2450_ACK_BUFFER_SIZE 128

/** @brief Size of the command buffer */
#define LD2450_CMD_BUFFER_SIZE 38

/** @brief Data frame size (fixed for this protocol) */
#define LD2450_DATA_FRAME_SIZE 30

/** @brief Timeout for configuration commands in milliseconds */
#define LD2450_CONFIG_TIMEOUT_MS 3000

/** @brief Timeout for module restart in milliseconds */
#define LD2450_RESTART_TIMEOUT_MS 3000

/** @brief Size of the UART RX buffer (replaces CONFIG_LD2450_UART_RX_BUF_SIZE) */
#define LD2450_UART_RX_BUF_SIZE 512

/** @brief Stack size for the processing task (replaces CONFIG_LD2450_TASK_STACK_SIZE) */
#define LD2450_TASK_STACK_SIZE 4096

/** @brief MIN macro for getting minimum of two values */
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/**
 * @brief Frame headers and footers
 */
/** @brief Data frame header (0xAA, 0xFF, 0x03, 0x00) */
static const uint8_t LD2450_DATA_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
/** @brief Data frame footer (0x55, 0xCC) */
static const uint8_t LD2450_DATA_FRAME_FOOTER[] = {0x55, 0xCC};
/** @brief Config frame header (0xFD, 0xFC, 0xFB, 0xFA) */
static const uint8_t LD2450_CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
/** @brief Config frame footer (0x04, 0x03, 0x02, 0x01) */
static const uint8_t LD2450_CONFIG_FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

/**
 * @brief Command words defined in the protocol
 */
typedef enum {
    /** @brief Enable configuration command (0x00FF) */
    LD2450_CMD_ENABLE_CONFIG    = 0x00FF,
    /** @brief End configuration command (0x00FE) */
    LD2450_CMD_END_CONFIG       = 0x00FE,
    /** @brief Set single target tracking mode (0x0080) */
    LD2450_CMD_SINGLE_TARGET    = 0x0080,
    /** @brief Set multi-target tracking mode (0x0090) */
    LD2450_CMD_MULTI_TARGET     = 0x0090,
    /** @brief Query target tracking mode (0x0091) */
    LD2450_CMD_QUERY_TARGET_MODE = 0x0091,
    /** @brief Read firmware version (0x00A0) */
    LD2450_CMD_READ_FW_VERSION  = 0x00A0,
    /** @brief Set serial port baud rate (0x00A1) */
    LD2450_CMD_SET_BAUD_RATE    = 0x00A1,
    /** @brief Restore factory settings (0x00A2) */
    LD2450_CMD_RESTORE_FACTORY  = 0x00A2,
    /** @brief Restart module (0x00A3) */
    LD2450_CMD_RESTART_MODULE   = 0x00A3,
    /** @brief Set Bluetooth settings (0x00A4) */
    LD2450_CMD_SET_BLUETOOTH    = 0x00A4,
    /** @brief Get MAC address (0x00A5) */
    LD2450_CMD_GET_MAC_ADDRESS  = 0x00A5,
    /** @brief Query region filtering configuration (0x00C1) */
    LD2450_CMD_QUERY_REGION     = 0x00C1,
    /** @brief Set region filtering configuration (0x00C2) */
    LD2450_CMD_SET_REGION       = 0x00C2
} ld2450_cmd_t;

/**
 * @brief Driver state structure
 */
typedef struct {
    /** @brief UART port number being used */
    uart_port_t uart_port;
    /** @brief RX pin number */
    int rx_pin;
    /** @brief TX pin number */
    int tx_pin;
    /** @brief Current baud rate */
    uint32_t baud_rate;
    /** @brief Driver initialized flag */
    bool initialized;
    /** @brief Auto-processing enabled flag */
    bool auto_processing;
    /** @brief Target data callback function */
    ld2450_target_cb_t target_callback;
    /** @brief User context for callback */
    void *user_ctx;
    /** @brief Processing task handle */
    TaskHandle_t task_handle;
    /** @brief UART event queue */
    QueueHandle_t uart_queue;
    /** @brief Mutex for thread safety */
    SemaphoreHandle_t mutex;
    /** @brief Protocol state */
    volatile bool in_config_mode;
    /** @brief Command buffer for sending commands */
    uint8_t cmd_buffer[LD2450_CMD_BUFFER_SIZE];
    /** @brief ACK buffer for receiving responses */
    uint8_t ack_buffer[LD2450_ACK_BUFFER_SIZE];
    /** @brief Data frame buffer */
    uint8_t frame_buffer[LD2450_DATA_FRAME_SIZE];
    /** @brief Frame buffer index */
    uint16_t frame_idx;
    /** @brief Frame synchronization state */
    bool frame_synced;
} ld2450_state_t;

/**
 * @brief Configuration functions
 */

/**
 * @brief Enter configuration mode
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_enter_config_mode(void);

/**
 * @brief Exit configuration mode
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_exit_config_mode(void);

/**
 * @brief Send a command to the LD2450 radar
 * 
 * @param cmd Command word
 * @param value Command value buffer
 * @param value_len Length of the command value
 * @param ack_buffer Buffer to store the ACK response
 * @param ack_len Pointer to store the length of the ACK response
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_send_command(ld2450_cmd_t cmd, const void *value, size_t value_len, 
                             uint8_t *ack_buffer, size_t *ack_len, uint32_t timeout_ms);

/**
 * @brief Parse data frame into frame structure
 * 
 * @param data Raw frame data
 * @param len Length of the data
 * @param frame Frame structure to fill
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_parse_frame(const uint8_t *data, size_t len, ld2450_frame_t *frame);

/**
 * @brief Task for processing radar data
 * 
 * @param arg Task argument (not used)
 */
void ld2450_processing_task(void *arg);

/**
 * @brief UART event handler
 * 
 * @param event UART event data
 */
void ld2450_uart_event_handler(void *arg);

/**
 * @brief Get driver instance (singleton)
 * 
 * @return Pointer to driver state structure
 */
ld2450_state_t *ld2450_get_instance(void);

/**
 * @brief Handle a complete data frame
 * 
 * @param data Frame data
 * @param len Frame length
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_handle_data_frame(const uint8_t *data, size_t len);

/**
 * @brief Validate ACK response
 * 
 * @param ack ACK buffer 
 * @param len ACK length
 * @param cmd Command word to validate against
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_validate_ack(const uint8_t *ack, size_t len, ld2450_cmd_t cmd);

#ifdef __cplusplus
}
#endif