/**
 * @file occupancy_manager.c
 * @brief Implementation of the occupancy manager component
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "occupancy_manager.h"

#define MAX_SOURCES 8
#define MAX_SOURCE_NAME_LEN 16

static const char *TAG = "occupancy_manager";

/* Source information structure */
typedef struct {
    char name[MAX_SOURCE_NAME_LEN];
    occupancy_reliability_t reliability;
    occupancy_source_type_t source_type;
    uint32_t timeout_ms;
    int64_t last_update_time;
    int count;
    bool is_certain;
    bool is_active;
    bool is_valid;
    bool has_ever_updated; /* To track if triggered sources have ever reported */
} source_info_t;

/* Callback information */
typedef struct {
    occupancy_change_cb_t callback;
    void *user_ctx;
    bool registered;
} callback_info_t;

/* Occupancy manager state */
static struct {
    source_info_t sources[MAX_SOURCES];
    occupancy_status_t current_status;
    callback_info_t callback;
    SemaphoreHandle_t lock;
    bool initialized;
} occupancy_ctx;

/**
 * @brief Check if a source has timed out
 */
static bool is_source_timed_out(const source_info_t *source) {
    if (source->timeout_ms == 0) {
        return false; // No timeout configured
    }
    
    // If it's a trigger-based source and it has never updated, don't consider it timed out
    if (source->source_type == OCCUPANCY_SOURCE_TRIGGERED && !source->has_ever_updated) {
        return false;
    }
    
    int64_t current_time = esp_timer_get_time() / 1000;
    int64_t elapsed = current_time - source->last_update_time;
    
    return elapsed > source->timeout_ms;
}

/**
 * @brief Update the current occupancy status based on all sources
 * @return true if status changed, false otherwise
 */
static bool update_occupancy_status(void) {
    occupancy_status_t old_status;
    memcpy(&old_status, &occupancy_ctx.current_status, sizeof(occupancy_status_t));
    
    // Start with no source and unknown count
    occupancy_ctx.current_status.count = -1;
    occupancy_ctx.current_status.is_occupied = false;
    occupancy_ctx.current_status.is_count_certain = false;
    occupancy_ctx.current_status.source = 0xFF;
    occupancy_ctx.current_status.determining_reliability = OCCUPANCY_RELIABILITY_LOW;
    
    // First check for occupied status starting from highest reliability
    for (int level = OCCUPANCY_RELIABILITY_ABSOLUTE; level >= OCCUPANCY_RELIABILITY_LOW; level--) {
        for (int i = 0; i < MAX_SOURCES; i++) {
            source_info_t *source = &occupancy_ctx.sources[i];
            
            // Skip invalid sources or sources with different reliability
            if (!source->is_valid || source->reliability != level) {
                continue;
            }
            
            // Skip inactive sources
            if (!source->is_active) {
                continue;
            }
            
            // For trigger-based sources that have never triggered, skip them
            if (source->source_type == OCCUPANCY_SOURCE_TRIGGERED && !source->has_ever_updated) {
                continue;
            }
            
            // Check if the source has timed out
            if (is_source_timed_out(source)) {
                // For continuous sources, we consider them no longer valid when timed out
                if (source->source_type == OCCUPANCY_SOURCE_CONTINUOUS) {
                    continue;
                }
                // For triggered sources, we still consider the last value valid
                // unless a long timeout is explicitly configured
                else if (source->timeout_ms > 60000) { // 1 minute threshold for triggered sources
                    continue;
                }
            }
            
            // Skip sources that don't know the count
            if (source->count < 0) {
                continue;
            }
            
            // If we found a source showing occupied and it's certain, use it
            if (source->count > 0 && source->is_certain) {
                occupancy_ctx.current_status.count = source->count;
                occupancy_ctx.current_status.is_occupied = true;
                occupancy_ctx.current_status.is_count_certain = true;
                occupancy_ctx.current_status.source = i;
                occupancy_ctx.current_status.determining_reliability = source->reliability;
                
                // We found an occupied state from the highest reliability, we can return
                return memcmp(&occupancy_ctx.current_status, &old_status, sizeof(occupancy_status_t)) != 0;
            }
            
            // If we found a source showing occupied but it's uncertain,
            // use it provisionally but continue looking for better sources
            if (source->count > 0 && !occupancy_ctx.current_status.is_count_certain) {
                occupancy_ctx.current_status.count = source->count;
                occupancy_ctx.current_status.is_occupied = true;
                occupancy_ctx.current_status.is_count_certain = false;
                occupancy_ctx.current_status.source = i;
                occupancy_ctx.current_status.determining_reliability = source->reliability;
            }
        }
        
        // If we found an occupied state at this reliability level, stop
        if (occupancy_ctx.current_status.is_occupied) {
            break;
        }
    }
    
    // If we didn't find any occupied status, use the highest reliability source showing unoccupied
    if (!occupancy_ctx.current_status.is_occupied) {
        for (int level = OCCUPANCY_RELIABILITY_ABSOLUTE; level >= OCCUPANCY_RELIABILITY_LOW; level--) {
            for (int i = 0; i < MAX_SOURCES; i++) {
                source_info_t *source = &occupancy_ctx.sources[i];
                
                // Skip invalid sources or sources with different reliability
                if (!source->is_valid || source->reliability != level) {
                    continue;
                }
                
                // Skip inactive sources
                if (!source->is_active) {
                    continue;
                }
                
                // For trigger-based sources that have never triggered, skip them
                if (source->source_type == OCCUPANCY_SOURCE_TRIGGERED && !source->has_ever_updated) {
                    continue;
                }
                
                // Check if the source has timed out
                if (is_source_timed_out(source)) {
                    // For continuous sources, we consider them no longer valid when timed out
                    if (source->source_type == OCCUPANCY_SOURCE_CONTINUOUS) {
                        continue;
                    }
                    // For triggered sources, we still consider the last value valid
                    // unless a long timeout is explicitly configured
                    else if (source->timeout_ms > 60000) { // 1 minute threshold for triggered sources
                        continue;
                    }
                }
                
                // Skip sources that don't know the count
                if (source->count < 0) {
                    continue;
                }
                
                // We found a source showing unoccupied
                occupancy_ctx.current_status.count = source->count;
                occupancy_ctx.current_status.is_occupied = false;
                occupancy_ctx.current_status.is_count_certain = source->is_certain;
                occupancy_ctx.current_status.source = i;
                occupancy_ctx.current_status.determining_reliability = source->reliability;
                
                // We found the highest reliability source with a count, stop
                return memcmp(&occupancy_ctx.current_status, &old_status, sizeof(occupancy_status_t)) != 0;
            }
        }
    }
    
    // Check if the status actually changed
    return memcmp(&occupancy_ctx.current_status, &old_status, sizeof(occupancy_status_t)) != 0;
}

/**
 * @brief Notify callback if registered and status changed
 * @param status_changed Whether the status has changed
 */
static void notify_if_changed(bool status_changed) {
    if (status_changed && occupancy_ctx.callback.registered) {
        // Make a copy of the status for the callback
        occupancy_status_t status_copy;
        memcpy(&status_copy, &occupancy_ctx.current_status, sizeof(occupancy_status_t));
        
        ESP_LOGI(TAG, "Status changed: occupied=%d, count=%d, source=%s(%d), reliability=%d", 
                status_copy.is_occupied, status_copy.count, 
                status_copy.source < MAX_SOURCES ? occupancy_ctx.sources[status_copy.source].name : "none",
                status_copy.source, status_copy.determining_reliability);
        
        // Call the callback with the status copy
        occupancy_ctx.callback.callback(&status_copy, occupancy_ctx.callback.user_ctx);
    }
}

/**
 * @brief Find a free source slot
 * @return index of free slot, or -1 if none available
 */
static int find_free_source_slot(void) {
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (!occupancy_ctx.sources[i].is_valid) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Validate source ID
 * @return true if valid, false if invalid
 */
static bool is_valid_source_id(occupancy_source_id_t id) {
    return (id < MAX_SOURCES) && occupancy_ctx.sources[id].is_valid;
}

/* Public API implementation */
esp_err_t occupancy_manager_init(void) {
    if (occupancy_ctx.initialized) {
        ESP_LOGW(TAG, "Occupancy manager already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize context
    memset(&occupancy_ctx, 0, sizeof(occupancy_ctx));
    
    // Create mutex
    occupancy_ctx.lock = xSemaphoreCreateMutex();
    if (occupancy_ctx.lock == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize current status
    occupancy_ctx.current_status.count = -1;
    occupancy_ctx.current_status.is_occupied = false;
    occupancy_ctx.current_status.is_count_certain = false;
    occupancy_ctx.current_status.source = 0xFF;
    occupancy_ctx.current_status.determining_reliability = OCCUPANCY_RELIABILITY_LOW;
    
    occupancy_ctx.initialized = true;
    ESP_LOGI(TAG, "Occupancy manager initialized");
    
    return ESP_OK;
}

esp_err_t occupancy_manager_deinit(void) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGW(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (occupancy_ctx.lock != NULL) {
        vSemaphoreDelete(occupancy_ctx.lock);
        occupancy_ctx.lock = NULL;
    }
    
    occupancy_ctx.initialized = false;
    ESP_LOGI(TAG, "Occupancy manager deinitialized");
    
    return ESP_OK;
}

esp_err_t occupancy_manager_register_source(const char *name, 
                                          occupancy_reliability_t reliability,
                                          occupancy_source_type_t source_type,
                                          uint32_t timeout_ms,
                                          int initial_count,
                                          occupancy_source_id_t *id) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (name == NULL || id == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    int slot = find_free_source_slot();
    if (slot < 0) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGE(TAG, "No free source slots available");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize source
    source_info_t *source = &occupancy_ctx.sources[slot];
    memset(source, 0, sizeof(source_info_t));
    
    // Copy name (truncate if necessary)
    strncpy(source->name, name, MAX_SOURCE_NAME_LEN - 1);
    source->name[MAX_SOURCE_NAME_LEN - 1] = '\0';
    
    source->reliability = reliability;
    source->source_type = source_type;
    source->timeout_ms = timeout_ms;
    source->count = initial_count;
    source->is_certain = (initial_count >= 0);
    source->is_active = true;
    source->is_valid = true;
    source->last_update_time = esp_timer_get_time() / 1000;
    source->has_ever_updated = (initial_count >= 0);
    
    // Assign ID
    *id = (occupancy_source_id_t)slot;
    
    // Update status with the new source
    bool status_changed = update_occupancy_status();
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    ESP_LOGI(TAG, "Registered source '%s' with ID %d, type %s, reliability %d", 
             name, slot, 
             (source_type == OCCUPANCY_SOURCE_CONTINUOUS) ? "continuous" : "triggered",
             reliability);
    
    // Notify if status changed
    notify_if_changed(status_changed);
    
    return ESP_OK;
}

esp_err_t occupancy_manager_unregister_source(occupancy_source_id_t id) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (!is_valid_source_id(id)) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGE(TAG, "Invalid source ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Mark source as invalid
    occupancy_ctx.sources[id].is_valid = false;
    
    ESP_LOGI(TAG, "Unregistered source '%s' with ID %d", 
             occupancy_ctx.sources[id].name, id);
    
    // Update status after removing the source
    bool status_changed = update_occupancy_status();
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    // Notify if status changed
    notify_if_changed(status_changed);
    
    return ESP_OK;
}

esp_err_t occupancy_manager_update_count(occupancy_source_id_t id, 
                                        int count, 
                                        bool is_certain) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (!is_valid_source_id(id)) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGE(TAG, "Invalid source ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    
    source_info_t *source = &occupancy_ctx.sources[id];
    
    // Update source information
    source->count = count;
    source->is_certain = is_certain;
    source->last_update_time = esp_timer_get_time() / 1000;
    source->has_ever_updated = true;
    
    ESP_LOGD(TAG, "Updated source '%s' (ID %d): count=%d, certain=%d", 
             source->name, id, count, is_certain);
    
    // Update status with the new count
    bool status_changed = update_occupancy_status();
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    // Notify if status changed
    notify_if_changed(status_changed);
    
    return ESP_OK;
}

esp_err_t occupancy_manager_trigger_presence(occupancy_source_id_t id, int min_count) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (!is_valid_source_id(id)) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGE(TAG, "Invalid source ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    
    source_info_t *source = &occupancy_ctx.sources[id];
    
    // For trigger sources, we just need to know there's someone present
    // We don't know the exact count, so we'll use min_count as a minimum
    int current_count = source->has_ever_updated ? source->count : 0;
    int new_count = (current_count > min_count) ? current_count : min_count;
    
    // Update source information
    source->count = new_count;
    source->is_certain = false;  // We're sure someone is there, but not exactly how many
    source->last_update_time = esp_timer_get_time() / 1000;
    source->has_ever_updated = true;
    
    ESP_LOGI(TAG, "Triggered presence on source '%s' (ID %d): count=%d", 
             source->name, id, new_count);
    
    // Update status with the new count
    bool status_changed = update_occupancy_status();
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    // Notify if status changed
    notify_if_changed(status_changed);
    
    return ESP_OK;
}

esp_err_t occupancy_manager_get_status(occupancy_status_t *status) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (status == NULL) {
        ESP_LOGE(TAG, "Invalid parameter: status is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Copy current status
    memcpy(status, &occupancy_ctx.current_status, sizeof(occupancy_status_t));
    
    // Add source name in log for debugging
    if (status->source != 0xFF) {
        ESP_LOGV(TAG, "Current status: occupied=%d, count=%d, certain=%d, source='%s'",
                status->is_occupied, status->count, status->is_count_certain, 
                occupancy_ctx.sources[status->source].name);
    }
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    return ESP_OK;
}

esp_err_t occupancy_manager_register_callback(occupancy_change_cb_t callback, void *user_ctx) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (callback == NULL) {
        ESP_LOGE(TAG, "Callback cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (occupancy_ctx.callback.registered) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGW(TAG, "Callback already registered, replacing");
    }
    
    // Register callback
    occupancy_ctx.callback.callback = callback;
    occupancy_ctx.callback.user_ctx = user_ctx;
    occupancy_ctx.callback.registered = true;
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    ESP_LOGI(TAG, "Status change callback registered");
    
    return ESP_OK;
}

esp_err_t occupancy_manager_unregister_callback(void) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Unregister callback
    occupancy_ctx.callback.registered = false;
    occupancy_ctx.callback.callback = NULL;
    occupancy_ctx.callback.user_ctx = NULL;
    
    xSemaphoreGive(occupancy_ctx.lock);
    
    ESP_LOGI(TAG, "Status change callback unregistered");
    
    return ESP_OK;
}

esp_err_t occupancy_manager_set_source_active(occupancy_source_id_t id, bool active) {
    if (!occupancy_ctx.initialized) {
        ESP_LOGE(TAG, "Occupancy manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(occupancy_ctx.lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    if (!is_valid_source_id(id)) {
        xSemaphoreGive(occupancy_ctx.lock);
        ESP_LOGE(TAG, "Invalid source ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Only update if state changes
    if (occupancy_ctx.sources[id].is_active != active) {
        occupancy_ctx.sources[id].is_active = active;
        
        ESP_LOGI(TAG, "Source '%s' (ID %d) set to %s", 
                 occupancy_ctx.sources[id].name, id, active ? "active" : "inactive");
        
        // Update status with the changed source state
        bool status_changed = update_occupancy_status();
        xSemaphoreGive(occupancy_ctx.lock);
        
        // Notify if status changed
        notify_if_changed(status_changed);
    } else {
        xSemaphoreGive(occupancy_ctx.lock);
    }
    
    return ESP_OK;
}