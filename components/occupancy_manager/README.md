# Occupancy Manager Component Documentation

## Table of Contents
1. [Overview](#overview)
2. [Key Features](#key-features)
3. [Architecture](#architecture)
4. [API Reference](#api-reference)
5. [Usage Examples](#usage-examples)
6. [Advanced Configuration](#advanced-configuration)
7. [Implementation Details](#implementation-details)

<a name="overview"></a>
## 1. Overview

The Occupancy Manager is an ESP-IDF component for ESP32 devices that provides reliable room occupancy detection by combining multiple data sources with different reliability levels. It's designed for smart home applications where accurate presence detection is crucial, such as automated lighting, HVAC control, and security systems.

The component addresses common challenges in occupancy detection:

- **Multi-source fusion**: Combines data from various sensors (radar, buttons, remotes) to improve accuracy
- **Reliability-based prioritization**: Gives precedence to more reliable sources when conflicts occur
- **Temporal handling**: Manages different update frequencies and supports timeout mechanisms
- **Certainty handling**: Distinguishes between certain and uncertain detections

<a name="key-features"></a>
## 2. Key Features

- Support for up to 8 concurrent occupancy data sources
- Hierarchical reliability levels (LOW → MEDIUM → HIGH → ABSOLUTE)
- Both continuous (e.g., radar) and triggered (e.g., buttons) source types
- Configurable timeouts for each data source
- Asynchronous callback notification system
- Source activity control (enable/disable sources without unregistering)
- Thread-safe implementation with mutex protection

<a name="architecture"></a>
## 3. Architecture

The Occupancy Manager follows a centralized architecture where multiple data sources register with the manager. Each source reports occupancy data which is then evaluated based on reliability level, source type, and timeout configuration.

```
┌────────────────────────────────────────┐
│         Application Layer              │
└───────────────────┬────────────────────┘
                    │
┌───────────────────▼────────────────────┐
│       Occupancy Manager Module         │
│  (Combines inputs with priority logic) │
└───┬───────────────┬─────────────┬──────┘
    │               │             │
┌───▼───┐     ┌─────▼────┐    ┌───▼───┐
│ Radar │     │ Physical │    │Remote │
│Module │     │ Buttons  │    │Control│
└───────┘     └──────────┘    └───────┘
```



### Data Flow:
1. Sources register with the manager providing reliability level and type
2. Sources periodically update occupancy count (continuous sources) or report events (triggered sources)
3. Manager evaluates all active sources to determine the current occupancy status
4. Changes in occupancy status trigger registered callbacks

### Reliability Levels:

| Level                          | Value | Description            | Example Sources                         |
| ------------------------------ | ----- | ---------------------- | --------------------------------------- |
| OCCUPANCY_RELIABILITY_LOW      | 0     | Least reliable sources | Radar sensors, ultrasonic sensors       |
| OCCUPANCY_RELIABILITY_MEDIUM   | 1     | Moderately reliable    | Remote controls, smartphone apps        |
| OCCUPANCY_RELIABILITY_HIGH     | 2     | Highly reliable        | Physical buttons, motion sensors        |
| OCCUPANCY_RELIABILITY_ABSOLUTE | 3     | Maximum reliability    | Safety systems, direct manual overrides |

### Source Types:

| Type                        | Description                                                  |
| --------------------------- | ------------------------------------------------------------ |
| OCCUPANCY_SOURCE_CONTINUOUS | Updates regularly (e.g., radar sensors providing continuous data) |
| OCCUPANCY_SOURCE_TRIGGERED  | Updates only on events (e.g., button presses, remote commands) |

<a name="api-reference"></a>
## 4. API Reference

### Initialization and Cleanup

```c
esp_err_t occupancy_manager_init(void);
```
Initializes the occupancy manager component. Must be called before any other functions.
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_deinit(void);
```
Deinitializes the occupancy manager and frees resources.
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

### Source Management

```c
esp_err_t occupancy_manager_register_source(
    const char *name, 
    occupancy_reliability_t reliability,
    occupancy_source_type_t source_type,
    uint32_t timeout_ms,
    int initial_count,
    occupancy_source_id_t *id
);
```
Registers a new occupancy data source with the manager.
- **Parameters**:
  - `name`: Name of the source (for debugging), max 15 characters
  - `reliability`: Reliability level of this source
  - `source_type`: Whether the source updates continuously or on triggers
  - `timeout_ms`: How long this source remains valid after an update (0 = indefinitely)
  - `initial_count`: Initial occupancy count (-1 if unknown)
  - `id`: Pointer to store the assigned source ID
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_unregister_source(occupancy_source_id_t id);
```
Unregisters a previously registered source.
- **Parameters**:
  - `id`: ID of the source to unregister
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_set_source_active(occupancy_source_id_t id, bool active);
```
Temporarily enables or disables a source without unregistering it.
- **Parameters**:
  - `id`: Source ID
  - `active`: Whether the source should be active (true) or inactive (false)
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

### Data Updates

```c
esp_err_t occupancy_manager_update_count(
    occupancy_source_id_t id, 
    int count, 
    bool is_certain
);
```
Updates the occupancy count from a particular source.
- **Parameters**:
  - `id`: Source ID
  - `count`: New count value (-1 if unknown)
  - `is_certain`: Whether the source is certain about this count
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_trigger_presence(
    occupancy_source_id_t id, 
    int min_count
);
```
Convenience function for simple presence triggers without exact counts.
- **Parameters**:
  - `id`: Source ID
  - `min_count`: Minimum number of people to report (typically 1)
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

### Status and Notifications

```c
esp_err_t occupancy_manager_get_status(occupancy_status_t *status);
```
Gets the current occupancy status.
- **Parameters**:
  - `status`: Pointer to store the current occupancy status
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_register_callback(
    occupancy_change_cb_t callback, 
    void *user_ctx
);
```
Registers a callback function that will be called when occupancy status changes.
- **Parameters**:
  - `callback`: Function to call when status changes
  - `user_ctx`: User context pointer to pass to the callback
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

```c
esp_err_t occupancy_manager_unregister_callback(void);
```
Unregisters a previously registered callback.
- **Returns**: `ESP_OK` on success, appropriate error code otherwise

### Data Structures

```c
typedef enum {
    OCCUPANCY_RELIABILITY_LOW = 0,     // Automated sensors (radar)
    OCCUPANCY_RELIABILITY_MEDIUM = 1,  // Remote control input
    OCCUPANCY_RELIABILITY_HIGH = 2,    // Physical buttons (direct human input)
    OCCUPANCY_RELIABILITY_ABSOLUTE = 3 // Critical inputs (e.g., safety systems)
} occupancy_reliability_t;
```
Defines reliability levels for occupancy data sources.

```c
typedef enum {
    OCCUPANCY_SOURCE_CONTINUOUS, // Source updates continuously (e.g. radar)
    OCCUPANCY_SOURCE_TRIGGERED   // Source updates only on events (e.g. buttons, remote)
} occupancy_source_type_t;
```
Defines data source update behavior.

```c
typedef struct {
    int count;                         // Estimated number of occupants
    bool is_occupied;                  // True if room is considered occupied
    bool is_count_certain;             // True if count is reliable
    occupancy_source_id_t source;      // Source ID that determined the current state
    occupancy_reliability_t determining_reliability; // Reliability of determining source
} occupancy_status_t;
```
Structure containing occupancy status information.

```c
typedef void (*occupancy_change_cb_t)(const occupancy_status_t *status, void *user_ctx);
```
Callback function type for occupancy status changes.

<a name="usage-examples"></a>
## 5. Usage Examples

### Basic Usage Example

```c
#include "occupancy_manager.h"
#include "esp_log.h"

static const char* TAG = "occupancy_example";

// Callback function for occupancy status changes
static void occupancy_change_callback(const occupancy_status_t *status, void *user_ctx) {
    ESP_LOGI(TAG, "Occupancy changed: %s, count: %d", 
             status->is_occupied ? "OCCUPIED" : "EMPTY", 
             status->count);

    // Your application logic for handling occupancy changes
    if (status->is_occupied) {
        // Handle occupied state (e.g., turn on lights)
    } else {
        // Handle empty state (e.g., turn off lights)
    }
}

void app_main(void) {
    // Initialize the occupancy manager
    ESP_ERROR_CHECK(occupancy_manager_init());
    
    // Register the occupancy change callback
    ESP_ERROR_CHECK(occupancy_manager_register_callback(occupancy_change_callback, NULL));
    
    // Register a radar sensor as a continuous, low-reliability source
    // with a 10-second timeout and initial unknown state
    occupancy_source_id_t radar_id;
    ESP_ERROR_CHECK(occupancy_manager_register_source(
        "radar", 
        OCCUPANCY_RELIABILITY_LOW,
        OCCUPANCY_SOURCE_CONTINUOUS,
        10000,  // 10 seconds timeout
        -1,     // Initial count unknown
        &radar_id
    ));
    
    // Register a button as a triggered, high-reliability source
    // with no timeout (manual trigger only)
    occupancy_source_id_t button_id;
    ESP_ERROR_CHECK(occupancy_manager_register_source(
        "button", 
        OCCUPANCY_RELIABILITY_HIGH,
        OCCUPANCY_SOURCE_TRIGGERED,
        0,      // No timeout
        0,      // Initial count = 0
        &button_id
    ));
    
    // Your application main loop
    while (1) {
        // Example: Update radar data periodically (e.g., from a sensor reading)
        int radar_count = get_radar_occupancy_count();  // Your sensor function
        ESP_ERROR_CHECK(occupancy_manager_update_count(
            radar_id,
            radar_count,
            false  // Radar is not 100% certain about exact count
        ));
        
        // Button handling would typically be in an interrupt or event handler
        // e.g., when a button is pressed:
        // ESP_ERROR_CHECK(occupancy_manager_trigger_presence(button_id, 1));
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Update radar data every second
    }
}
```

### Multiple Sources Example

```c
// Register multiple sources with different reliability levels
occupancy_source_id_t radar_id;
ESP_ERROR_CHECK(occupancy_manager_register_source(
    "radar", 
    OCCUPANCY_RELIABILITY_LOW,
    OCCUPANCY_SOURCE_CONTINUOUS,
    10000,  // 10 seconds timeout
    -1,     // Initial count unknown
    &radar_id
));

occupancy_source_id_t remote_id;
ESP_ERROR_CHECK(occupancy_manager_register_source(
    "remote", 
    OCCUPANCY_RELIABILITY_MEDIUM,
    OCCUPANCY_SOURCE_TRIGGERED,
    60000,  // 1 minute timeout
    0,      // Initial count = 0 (no one)
    &remote_id
));

occupancy_source_id_t button_id;
ESP_ERROR_CHECK(occupancy_manager_register_source(
    "button", 
    OCCUPANCY_RELIABILITY_HIGH,
    OCCUPANCY_SOURCE_TRIGGERED,
    0,      // No timeout
    0,      // Initial count = 0
    &button_id
));

// In your application:

// Remote control signals presence
ESP_ERROR_CHECK(occupancy_manager_trigger_presence(remote_id, 1));

// Button overrides with higher reliability
ESP_ERROR_CHECK(occupancy_manager_update_count(button_id, 2, true));

// Get current status
occupancy_status_t status;
ESP_ERROR_CHECK(occupancy_manager_get_status(&status));

// Later, temporarily disable the radar source
ESP_ERROR_CHECK(occupancy_manager_set_source_active(radar_id, false));
```

<a name="advanced-configuration"></a>
## 6. Advanced Configuration

### Timeout Strategies

Timeouts are handled differently based on source type:
- **Continuous sources**: When timed out, the source is considered invalid until updated again
- **Triggered sources**: When timed out, the source remains valid unless a long timeout (>60s) is configured

Recommended timeout values:
- Radar/Ultrasonic: 5-15 seconds
- IR sensors: 1-5 minutes
- Remote controls: 1-5 minutes
- Buttons: 0 (never timeout) or very long (hours)

### Reliability Configuration Matrix

| Source Type     | Scenario           | Recommended Reliability | Timeout    |
| --------------- | ------------------ | ----------------------- | ---------- |
| Radar           | General use        | LOW                     | 5-15s      |
| PIR             | General use        | LOW-MEDIUM              | 1-5min     |
| mmWave          | Enhanced precision | MEDIUM                  | 10-30s     |
| Remote control  | User input         | MEDIUM                  | 1-5min     |
| Physical button | User presence      | HIGH                    | 0 or hours |
| Door sensor     | Direct evidence    | HIGH                    | 0 or hours |
| Safety system   | Critical override  | ABSOLUTE                | varies     |

### Memory Usage

The occupancy manager uses approximately:
- Base size: ~100 bytes
- Per source: ~36 bytes
- Total with 8 sources: ~388 bytes

<a name="implementation-details"></a>
## 7. Implementation Details

### Decision Logic

The occupancy determination algorithm uses the following logic:

1. Check for occupied status from highest reliability level (ABSOLUTE) to lowest (LOW)
   - First source showing occupied AND certain wins
   - If only uncertain occupation sources exist, the highest reliability one is used

2. If no source indicates occupation, use the highest reliability source showing unoccupied

3. Sources are only considered if:
   - They are registered and active
   - They haven't timed out (based on their timeout configuration)
   - For triggered sources, they must have been updated at least once

### Thread Safety

All public API functions are thread-safe and protected by a mutex. This allows:
- Calling functions from different tasks/threads
- Updating sources from interrupt contexts (with appropriate precautions)
- Safe callback execution

### Error Handling

The component uses ESP-IDF's error handling mechanism with `esp_err_t` return values:
- `ESP_OK`: Operation successful
- `ESP_ERR_INVALID_STATE`: Component not initialized
- `ESP_ERR_INVALID_ARG`: Invalid function arguments
- `ESP_ERR_NO_MEM`: No available source slots
- `ESP_ERR_TIMEOUT`: Mutex timeout (should not happen in normal operation)

### Logging

The component uses ESP-IDF's logging system with the tag "occupancy_manager":
- ERROR: Critical failures
- WARN: Non-critical issues
- INFO: Status changes and major operations
- DEBUG: Count updates and detailed operations
- VERBOSE: Verbose status information