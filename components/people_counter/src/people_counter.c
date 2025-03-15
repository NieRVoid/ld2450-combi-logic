/**
 * @file people_counter.c
 * @brief Implementation of people counting module using LD2450 radar sensor
 *
 * This module implements a people counting algorithm that monitors x-axis
 * movements to determine entries and exits through a doorway.
 *
 * @author NieRVoid
 * @date 2025-03-14
 * @license MIT
 */

#include <string.h>
#include "people_counter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "PEOPLE_COUNTER";

/**
 * @brief Target tracking structure
 */
typedef struct {
    int16_t initial_x;          // Initial X position when target first detected
    int16_t current_x;          // Current X position
    int16_t vector_length;      // Current vector length (current_x - initial_x)
    bool counted;               // Flag indicating if this trajectory has been counted
    bool active;                // Flag indicating if target is currently active
    uint8_t empty_count;        // Counter for consecutive empty reports
    int64_t first_seen_time;    // Timestamp when target first detected (µs)
    int64_t last_update_time;   // Last update timestamp (µs)
} pc_target_t;

/**
 * @brief Module context structure
 */
typedef struct {
    int count;                  // Current people count (entries - exits)
    int total_entries;          // Total entries counted
    int total_exits;            // Total exits counted
    pc_target_t targets[3];     // Array of tracked targets (max 3 from LD2450)
    people_counter_config_t config;
    bool initialized;
    SemaphoreHandle_t mutex;    // For thread safety
} people_counter_context_t;

// Global instance of the counter context
static people_counter_context_t s_pc_context = {0};

// Forward declarations of static functions
static void radar_data_callback(const ld2450_frame_t *frame, void *ctx);
static void process_target_movement(pc_target_t *target);
static bool is_in_detection_area(const ld2450_target_t *target, const people_counter_config_t *config);

/**
 * @brief Initialize people counter with configuration
 */
esp_err_t people_counter_init(const people_counter_config_t *config)
{
    esp_err_t ret;

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_pc_context.initialized) {
        ESP_LOGW(TAG, "People counter already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize context with defaults
    memset(&s_pc_context, 0, sizeof(s_pc_context));
    memcpy(&s_pc_context.config, config, sizeof(people_counter_config_t));

    // Create mutex for thread safety
    s_pc_context.mutex = xSemaphoreCreateMutex();
    if (s_pc_context.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Register callback for radar data
    ret = ld2450_register_target_callback(radar_data_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register radar callback: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_pc_context.mutex);
        return ret;
    }

    // Configure detection regions for the radar
    ret = people_counter_configure_region();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure detection region: %s", esp_err_to_name(ret));
        // Continue anyway as this is not critical
    }

    s_pc_context.initialized = true;
    ESP_LOGI(TAG, "People counter initialized with vector threshold: %d mm", config->vector_threshold);

    return ESP_OK;
}

/**
 * @brief Configure detection region for radar
 */
esp_err_t people_counter_configure_region(void)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Configure detection regions for the radar
    ld2450_region_t regions[3] = {
        {
            .x1 = s_pc_context.config.detection_min_x,
            .y1 = s_pc_context.config.detection_min_y,
            .x2 = s_pc_context.config.detection_max_x,
            .y2 = s_pc_context.config.detection_max_y
        },
        {0, 0, 0, 0}, // Not used
        {0, 0, 0, 0}  // Not used
    };

    return ld2450_set_region_filter(LD2450_FILTER_INCLUDE_ONLY, regions);
}

/**
 * @brief Deinitialize and release resources
 */
esp_err_t people_counter_deinit(void)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Unregister callback
    ld2450_register_target_callback(NULL, NULL);

    // Delete mutex
    if (s_pc_context.mutex != NULL) {
        vSemaphoreDelete(s_pc_context.mutex);
        s_pc_context.mutex = NULL;
    }

    s_pc_context.initialized = false;
    ESP_LOGI(TAG, "People counter deinitialized");

    return ESP_OK;
}

/**
 * @brief Update configuration parameters
 */
esp_err_t people_counter_update_config(const people_counter_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_pc_context.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Update configuration parameters
    s_pc_context.config.vector_threshold = config->vector_threshold;
    s_pc_context.config.empty_target_threshold = config->empty_target_threshold;
    s_pc_context.config.detection_min_x = config->detection_min_x;
    s_pc_context.config.detection_max_x = config->detection_max_x;
    s_pc_context.config.detection_min_y = config->detection_min_y;
    s_pc_context.config.detection_max_y = config->detection_max_y;
    
    // Update callback if provided
    if (config->count_changed_cb != NULL) {
        s_pc_context.config.count_changed_cb = config->count_changed_cb;
        s_pc_context.config.user_context = config->user_context;
    }

    xSemaphoreGive(s_pc_context.mutex);

    // Update region filter if detection area changed
    esp_err_t ret = people_counter_configure_region();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update region filter: %s", esp_err_to_name(ret));
        // Continue anyway as this is not critical
    }

    ESP_LOGI(TAG, "Configuration updated with vector threshold: %d mm", config->vector_threshold);
    return ESP_OK;
}

/**
 * @brief Get current people count
 */
int people_counter_get_count(void)
{
    if (!s_pc_context.initialized) {
        ESP_LOGW(TAG, "People counter not initialized");
        return 0;
    }

    int count;
    if (xSemaphoreTake(s_pc_context.mutex, portMAX_DELAY) == pdTRUE) {
        count = s_pc_context.count;
        xSemaphoreGive(s_pc_context.mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        count = 0;
    }

    return count;
}

/**
 * @brief Get detailed counts
 */
esp_err_t people_counter_get_details(int *count, int *entries, int *exits)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (count == NULL && entries == NULL && exits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_pc_context.mutex, portMAX_DELAY) == pdTRUE) {
        if (count != NULL) {
            *count = s_pc_context.count;
        }
        if (entries != NULL) {
            *entries = s_pc_context.total_entries;
        }
        if (exits != NULL) {
            *exits = s_pc_context.total_exits;
        }
        xSemaphoreGive(s_pc_context.mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Reset counter to zero
 */
esp_err_t people_counter_reset(void)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_pc_context.mutex, portMAX_DELAY) == pdTRUE) {
        s_pc_context.count = 0;
        s_pc_context.total_entries = 0;
        s_pc_context.total_exits = 0;
        
        // Reset all target tracking
        for (int i = 0; i < 3; i++) {
            s_pc_context.targets[i].active = false;
            s_pc_context.targets[i].counted = false;
            s_pc_context.targets[i].empty_count = 0;
        }
        
        xSemaphoreGive(s_pc_context.mutex);
        
        ESP_LOGI(TAG, "People counter reset to zero");
        
        // Notify via callback if registered
        if (s_pc_context.config.count_changed_cb != NULL) {
            s_pc_context.config.count_changed_cb(
                0, 0, 0,
                s_pc_context.config.user_context
            );
        }
        
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Register callback for count changes
 */
esp_err_t people_counter_register_callback(
    void (*callback)(int count, int entries, int exits, void *context), 
    void *user_context)
{
    if (!s_pc_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_pc_context.mutex, portMAX_DELAY) == pdTRUE) {
        s_pc_context.config.count_changed_cb = callback;
        s_pc_context.config.user_context = user_context;
        xSemaphoreGive(s_pc_context.mutex);
        
        ESP_LOGI(TAG, "Callback %sregistered", callback ? "" : "un");
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Check if target is in detection area
 */
static bool is_in_detection_area(const ld2450_target_t *target, const people_counter_config_t *config)
{
    return (target->x >= config->detection_min_x && 
            target->x <= config->detection_max_x &&
            target->y >= config->detection_min_y && 
            target->y <= config->detection_max_y);
}

/**
 * @brief Process target movement to detect entry or exit
 */
static void process_target_movement(pc_target_t *target)
{
    if (target->counted || !target->active) {
        return; // Already counted or inactive
    }
    
    // Calculate vector length (difference between current and initial position)
    target->vector_length = target->current_x - target->initial_x;
    
    // Check if vector length exceeds threshold
    if (abs(target->vector_length) >= s_pc_context.config.vector_threshold) {
        if (target->vector_length > 0) {
            // Positive direction = entry
            s_pc_context.total_entries++;
            s_pc_context.count++;
            ESP_LOGI(TAG, "Entry detected. Current count: %d people in room", s_pc_context.count);
        } else {
            // Negative direction = exit
            s_pc_context.total_exits++;
            s_pc_context.count--;
            ESP_LOGI(TAG, "Exit detected. Current count: %d people in room", s_pc_context.count);
        }
        
        // Mark as counted to avoid duplicate counts
        target->counted = true;
        
        // Call callback if registered
        if (s_pc_context.config.count_changed_cb) {
            s_pc_context.config.count_changed_cb(
                s_pc_context.count,
                s_pc_context.total_entries,
                s_pc_context.total_exits,
                s_pc_context.config.user_context
            );
        }
    }
}

/**
 * @brief Callback function to receive radar data
 */
static void radar_data_callback(const ld2450_frame_t *frame, void *ctx)
{
    if (!s_pc_context.initialized || frame == NULL) {
        return;
    }

    if (xSemaphoreTake(s_pc_context.mutex, 0) != pdTRUE) {
        ESP_LOGV(TAG, "Skipping frame processing - mutex busy");
        return;
    }

    // Process each target in the frame
    for (int i = 0; i < 3; i++) {
        pc_target_t *target = &s_pc_context.targets[i];
        
        // Check if target is valid in the current frame
        bool valid_in_frame = (i < frame->count) && 
                              frame->targets[i].valid && 
                              is_in_detection_area(&frame->targets[i], &s_pc_context.config);
        
        if (valid_in_frame) {
            const ld2450_target_t *radar_target = &frame->targets[i];
            
            // Target is active in this frame
            if (!target->active) {
                // New target appeared
                target->active = true;
                target->initial_x = radar_target->x;
                target->current_x = radar_target->x;
                target->first_seen_time = frame->timestamp;
                target->counted = false;
                target->empty_count = 0;
                target->vector_length = 0;
                
                ESP_LOGD(TAG, "New target %d detected at position (%d, %d)", 
                        i, radar_target->x, radar_target->y);
            } else {
                // Update target position
                target->current_x = radar_target->x;
                target->last_update_time = frame->timestamp;
                target->empty_count = 0;
                
                // Process movement for entry/exit detection
                process_target_movement(target);
                
                ESP_LOGV(TAG, "Target %d updated: pos=(%d,%d), vector=%d, counted=%d", 
                        i, radar_target->x, radar_target->y, target->vector_length, target->counted);
            }
        } else if (target->active) {
            // Target was active but now missing
            target->empty_count++;
            
            if (target->empty_count >= s_pc_context.config.empty_target_threshold) {
                // Reset target tracking
                ESP_LOGD(TAG, "Target %d lost after %d empty frames", i, target->empty_count);
                target->active = false;
            }
        }
    }
    
    xSemaphoreGive(s_pc_context.mutex);
}