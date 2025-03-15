/**
 * @file occupancy_manager.h
 * @brief Manages multiple occupancy data sources with reliability-based logic
 * 
 * This component combines data from multiple occupancy detection sources with
 * different reliability levels to determine the most likely room occupancy status.
 * Sources can report data at different rates - some continuously (like radar),
 * others only occasionally (like buttons or remote controls).
 * 
 * @author NieRVoid
 * @date 2025-03-15
 * @license MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reliability levels for occupancy data sources
 */
typedef enum {
    OCCUPANCY_RELIABILITY_LOW = 0,     /*!< Automated sensors (radar) */
    OCCUPANCY_RELIABILITY_MEDIUM = 1,  /*!< Remote control input */
    OCCUPANCY_RELIABILITY_HIGH = 2,    /*!< Physical buttons (direct human input) */
    OCCUPANCY_RELIABILITY_ABSOLUTE = 3 /*!< Critical inputs (e.g., safety systems) */
} occupancy_reliability_t;

/**
 * @brief Source ID type
 */
typedef uint8_t occupancy_source_id_t;

/**
 * @brief Data source update behavior
 */
typedef enum {
    OCCUPANCY_SOURCE_CONTINUOUS, /*!< Source updates continuously (e.g. radar) */
    OCCUPANCY_SOURCE_TRIGGERED   /*!< Source updates only on events (e.g. buttons, remote) */
} occupancy_source_type_t;

/**
 * @brief Occupancy status information
 */
typedef struct {
    int count;                      /*!< Estimated number of occupants */
    bool is_occupied;               /*!< True if room is considered occupied */
    bool is_count_certain;          /*!< True if count is reliable */
    occupancy_source_id_t source;   /*!< Source ID that determined the current state */
    occupancy_reliability_t determining_reliability; /*!< Reliability of determining source */
} occupancy_status_t;

/**
 * @brief Initialize the occupancy manager
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_init(void);

/**
 * @brief Deinitialize the occupancy manager
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_deinit(void);

/**
 * @brief Register a new occupancy data source
 * 
 * @param name Name of the source (for debugging)
 * @param reliability Reliability level of this source
 * @param source_type The update behavior of this source
 * @param timeout_ms How long this source is valid after update (0 for indefinitely)
 * @param initial_count Initial occupancy count from this source (-1 if unknown)
 * @param id Pointer to store the assigned source ID
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_register_source(const char *name, 
                                          occupancy_reliability_t reliability,
                                          occupancy_source_type_t source_type,
                                          uint32_t timeout_ms,
                                          int initial_count,
                                          occupancy_source_id_t *id);

/**
 * @brief Unregister an occupancy data source
 * 
 * @param id ID of the source to unregister
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_unregister_source(occupancy_source_id_t id);

/**
 * @brief Update count from a particular source
 * 
 * This function is typically called when an event occurs for trigger-based sources
 * (like button press or remote command), or periodically for continuous sources
 * (like radar).
 * 
 * @param id Source ID
 * @param count New count value (-1 if unknown)
 * @param is_certain Whether the source is certain about this count
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_update_count(occupancy_source_id_t id, 
                                        int count, 
                                        bool is_certain);

/**
 * @brief Handle occupancy presence trigger from a simple source
 * 
 * This is a convenience function for trigger-based sources like buttons
 * that only indicate presence without providing a specific count.
 * 
 * @param id Source ID
 * @param min_count Minimum number of people to report (typically 1)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_trigger_presence(occupancy_source_id_t id, int min_count);

/**
 * @brief Get current occupancy status
 * 
 * @param status Pointer to store the current occupancy status
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_get_status(occupancy_status_t *status);

/**
 * @brief Type definition for occupancy status change callback
 */
typedef void (*occupancy_change_cb_t)(const occupancy_status_t *status, void *user_ctx);

/**
 * @brief Register a callback for status changes
 * 
 * @param callback Function to call when status changes
 * @param user_ctx User context to pass to the callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_register_callback(occupancy_change_cb_t callback, void *user_ctx);

/**
 * @brief Unregister previously registered callback
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_unregister_callback(void);

/**
 * @brief Set source active state
 * 
 * @param id Source ID
 * @param active Whether the source is active
 * @return esp_err_t ESP_OK on success
 */
esp_err_t occupancy_manager_set_source_active(occupancy_source_id_t id, bool active);

#ifdef __cplusplus
}
#endif