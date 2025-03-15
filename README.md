# People Counter Component

## Overview

The People Counter component is an ESP-IDF component that uses the HLK-LD2450 24GHz radar sensor to count people entering and exiting a room or doorway. It tracks the movement of targets along the X-axis and determines entries and exits based on the direction and magnitude of movement.

This component relies on the `ld2450_driver` component to interface with the hardware radar sensor and processes the radar data to maintain a count of people in the monitored area.

## Features

- **People counting:** Accurately counts people entering and exiting through a doorway
- **Direction detection:** Distinguishes between entries (positive X movement) and exits (negative X movement)
- **Configurable detection area:** Customizable boundaries for target detection
- **Movement thresholds:** Adjustable parameters for counting sensitivity
- **Event notification:** Optional callback mechanism for count changes
- **Thread-safe design:** Uses mutexes to ensure thread safety in multithreaded environments

## Hardware Requirements

- ESP32 microcontroller (compatible with ESP-IDF v5.4)
- HLK-LD2450 24GHz radar sensor module

## Installation

Place the component in your ESP-IDF project's `components` directory:

```
your-project/
├── components/
│   ├── people_counter/
│   │   ├── include/
│   │   │   └── people_counter.h
│   │   ├── src/
│   │   │   └── people_counter.c
│   │   └── CMakeLists.txt
│   └── ld2450_driver/  (required dependency)
└── main/
    └── app_main.c
```

Add the following line to your project's `CMakeLists.txt` to include the component:

```cmake
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)
```

## Usage

### Initialization

First, initialize the LD2450 driver component, then initialize the People Counter component:

```c
#include "ld2450.h"
#include "people_counter.h"

void app_main(void)
{
    // Initialize LD2450 radar sensor
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(ld2450_init(&radar_config));
    
    // Initialize people counter with default configuration
    people_counter_config_t counter_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(people_counter_init(&counter_config));
    
    // Rest of your application
}
```

### Configuration Parameters

The people counter can be configured using the `people_counter_config_t` structure:

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `vector_threshold` | X-axis movement required to count entry/exit (mm) | 1500 (150cm) |
| `empty_target_threshold` | Number of empty reports before target is considered gone | 10 |
| `detection_min_x` | Minimum X-coordinate of detection area (mm) | -1500 (-150cm) |
| `detection_max_x` | Maximum X-coordinate of detection area (mm) | 1500 (150cm) |
| `detection_min_y` | Minimum Y-coordinate of detection area (mm) | 0 |
| `detection_max_y` | Maximum Y-coordinate of detection area (mm) | 2000 (200cm) |
| `count_changed_cb` | Callback function pointer for count changes | NULL |
| `user_context` | User context passed to callback | NULL |

### Reading Count Values

You can retrieve the current people count using the provided functions:

```c
// Get only the current count
int count = people_counter_get_count();
printf("Current people count: %d\n", count);

// Get detailed statistics
int count, entries, exits;
ESP_ERROR_CHECK(people_counter_get_details(&count, &entries, &exits));
printf("People count: %d (Entries: %d, Exits: %d)\n", count, entries, exits);
```

### Event Callbacks

Register a callback function to be notified when the people count changes:

```c
void count_changed_handler(int count, int entries, int exits, void *context)
{
    printf("Count changed! Current count: %d (Entries: %d, Exits: %d)\n", 
           count, entries, exits);
    
    // Your application logic here
}

// Register callback
ESP_ERROR_CHECK(people_counter_register_callback(count_changed_handler, NULL));
```

### Updating Configuration

You can update the configuration parameters at runtime:

```c
people_counter_config_t new_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
new_config.vector_threshold = 1000;  // Reduce required movement to 1 meter
new_config.detection_max_y = 3000;   // Increase depth to 3 meters

ESP_ERROR_CHECK(people_counter_update_config(&new_config));
```

### Resetting the Counter

To reset the counters to zero:

```c
ESP_ERROR_CHECK(people_counter_reset());
```

### Deinitialization

```c
// Deinitialize in reverse order
ESP_ERROR_CHECK(people_counter_deinit());
ESP_ERROR_CHECK(ld2450_deinit());
```

## Implementation Details

### Detection Algorithm

The people counter component uses a vector-based approach to detect entries and exits:

1. When a radar target is first detected, its initial X position is recorded
2. As the target moves, the component calculates a vector (current_x - initial_x)
3. When the vector exceeds the configured threshold:
   - If the vector is positive (moving right), an entry is counted
   - If the vector is negative (moving left), an exit is counted

### Target Tracking

The component can track up to 3 targets simultaneously (the maximum supported by the LD2450 sensor). For each target, it maintains:

- Initial X position when target first detected
- Current X position
- Vector length (movement distance)
- Target status (active, counted)
- Timestamps for first detection and last update

### Thread Safety

The component is designed to be thread-safe through the use of a FreeRTOS mutex. All operations that access or modify the internal state are protected by this mutex.

## Example Application

```c
#include "esp_log.h"
#include "ld2450.h"
#include "people_counter.h"

static const char *TAG = "PEOPLE_COUNTER_EXAMPLE";

void count_change_callback(int count, int entries, int exits, void *context)
{
    ESP_LOGI(TAG, "People count changed: %d people in room (Entries: %d, Exits: %d)",
             count, entries, exits);
}

void app_main(void)
{
    // Initialize radar sensor
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(ld2450_init(&radar_config));
    
    // Configure people counter
    people_counter_config_t counter_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    counter_config.vector_threshold = 1200;  // 1.2m movement threshold
    counter_config.count_changed_cb = count_change_callback;
    
    ESP_ERROR_CHECK(people_counter_init(&counter_config));
    
    ESP_LOGI(TAG, "People counter initialized and running");
    
    // Application continues running while the counter works in the background
}
```

## Configuration Tips

- **Detection Area**: Configure the detection area to match the doorway width and depth. The X-axis should be aligned with the doorway width.
- **Vector Threshold**: Set the vector threshold to approximately the width of the doorway to ensure people are counted only when they fully pass through.
- **Empty Target Threshold**: Adjusts how quickly the component forgets about targets that disappear. Higher values improve tracking continuity but may cause issues with rapid movements.

## Limitations

- The component can only track up to 3 targets simultaneously (limitation of the LD2450 sensor)
- Accuracy may be reduced when multiple people cross the doorway at the same time
- The component assumes the radar is positioned so that the X-axis is aligned with the doorway

## API Reference

### Functions

#### people_counter_init
```c
esp_err_t people_counter_init(const people_counter_config_t *config);
```
Initializes the people counter with the provided configuration.

#### people_counter_deinit
```c
esp_err_t people_counter_deinit(void);
```
Deinitializes the people counter and releases resources.

#### people_counter_update_config
```c
esp_err_t people_counter_update_config(const people_counter_config_t *config);
```
Updates the configuration parameters at runtime.

#### people_counter_configure_region
```c
esp_err_t people_counter_configure_region(void);
```
Configures detection region for radar based on current configuration.

#### people_counter_get_count
```c
int people_counter_get_count(void);
```
Returns the current people count (entries minus exits).

#### people_counter_get_details
```c
esp_err_t people_counter_get_details(int *count, int *entries, int *exits);
```
Gets detailed counts including total entries and exits.

#### people_counter_reset
```c
esp_err_t people_counter_reset(void);
```
Resets all counters to zero.

#### people_counter_register_callback
```c
esp_err_t people_counter_register_callback(
    void (*callback)(int count, int entries, int exits, void *context), 
    void *user_context);
```
Registers a callback function to be called when the count changes.

### Structures

#### people_counter_config_t
```c
typedef struct {
    int16_t vector_threshold;       // X-axis movement required to count entry/exit (mm)
    uint8_t empty_target_threshold; // Number of empty reports before target is considered gone
    int16_t detection_min_x;        // Detection area boundaries (mm)
    int16_t detection_max_x;
    int16_t detection_min_y;
    int16_t detection_max_y;
    void (*count_changed_cb)(int count, int entries, int exits, void* context);
    void* user_context;
} people_counter_config_t;
```

## Troubleshooting

### Common Issues

1. **People not being counted**:
   - Check if the radar is positioned correctly
   - Verify the detection area configuration matches the doorway dimensions
   - Ensure the vector threshold is appropriate for the doorway width

2. **False counts**:
   - Increase the vector threshold
   - Check if the detection area is properly configured

3. **Duplicate counts**:
   - The algorithm is designed to prevent duplicate counts, but if they occur, try adjusting the empty target threshold

### Debug Logging

The component uses ESP-IDF's logging mechanism with the tag "PEOPLE_COUNTER". You can enable different log levels in the menuconfig to aid in troubleshooting.

## License

This component is released under the MIT license.