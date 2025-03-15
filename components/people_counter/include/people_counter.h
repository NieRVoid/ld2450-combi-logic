/**
 * @file people_counter.h
 * @brief People counting module using LD2450 radar sensor
 *
 * This module counts people entering and exiting a room by tracking movement
 * along the x-axis using the HLK-LD2450 24GHz radar sensor.
 *
 * @author NieRVoid
 * @date 2025-03-14
 * @license MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ld2450.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for the people counter
 */
typedef struct {
    // Detection thresholds
    int16_t vector_threshold;       // X-axis movement required to count entry/exit (mm)
    uint8_t empty_target_threshold; // Number of empty reports before target is considered gone
    
    // Detection area configuration
    int16_t detection_min_x;        // Detection area boundaries (mm)
    int16_t detection_max_x;
    int16_t detection_min_y;
    int16_t detection_max_y;
    
    // Optional callback
    void (*count_changed_cb)(int count, int entries, int exits, void* context);
    void* user_context;
} people_counter_config_t;

/**
 * @brief Default configuration for the people counter
 */
#define PEOPLE_COUNTER_DEFAULT_CONFIG() { \
    .vector_threshold = 1500,         /* 150cm movement required to count */ \
    .empty_target_threshold = 10,     /* 10 consecutive empty reports to consider target gone */ \
    .detection_min_x = -1500,         /* Detection area: 3m wide, 2m deep */ \
    .detection_max_x = 1500, \
    .detection_min_y = 0, \
    .detection_max_y = 2000, \
    .count_changed_cb = NULL, \
    .user_context = NULL \
}

/**
 * @brief Initialize people counter with externally initialized radar driver
 * 
 * @note The radar driver must be initialized before calling this function
 * 
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_init(const people_counter_config_t *config);

/**
 * @brief Deinitialize the people counter
 * 
 * @note This does not deinitialize the radar driver, which should be handled by the application
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_deinit(void);

/**
 * @brief Update configuration parameters
 * 
 * @param config Pointer to new configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_update_config(const people_counter_config_t *config);

/**
 * @brief Configure detection region for radar
 * 
 * @note This function updates the radar's region filtering settings
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_configure_region(void);

/**
 * @brief Get current people count (number of people in the room)
 * 
 * @return int Current count (negative if more exits than entries)
 */
int people_counter_get_count(void);

/**
 * @brief Get detailed counts
 * 
 * @param count Pointer to store current count
 * @param entries Pointer to store total entries
 * @param exits Pointer to store total exits
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_get_details(int *count, int *entries, int *exits);

/**
 * @brief Reset counter to zero
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_reset(void);

/**
 * @brief Register callback for count changes
 * 
 * @param callback Function to call when count changes
 * @param user_context User context passed to callback
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t people_counter_register_callback(
    void (*callback)(int count, int entries, int exits, void *context), 
    void *user_context);

#ifdef __cplusplus
}
#endif