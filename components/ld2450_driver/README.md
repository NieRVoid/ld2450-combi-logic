# HLK-LD2450 Driver Documentation
## ESP-IDF Component for 24GHz Radar Detection

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Hardware Overview](#hardware-overview)
4. [Installation](#installation)
5. [API Reference](#api-reference)
   - [Initialization](#initialization)
   - [Data Reception](#data-reception)
   - [Configuration Commands](#configuration-commands)
   - [Target and Region Handling](#target-and-region-handling)
6. [Usage Examples](#usage-examples)
   - [Basic Initialization](#basic-initialization)
   - [Target Detection](#target-detection)
   - [Region Filtering](#region-filtering)
7. [Data Structures](#data-structures)
8. [Troubleshooting](#troubleshooting)
9. [License](#license)

## Introduction

The HLK-LD2450 Driver is an ESP-IDF component that provides a complete interface to control and receive data from the HLK-LD2450 24GHz millimeter-wave radar sensor. This radar module is capable of detecting human presence and tracking multiple targets simultaneously with high precision. It uses FMCW (Frequency-Modulated Continuous Wave) technology, making it suitable for various applications like presence detection, people counting, and motion tracking.

The driver handles all the communication protocol details, data parsing, and provides a clean abstraction layer for applications to interact with the radar sensor. It's designed with memory efficiency in mind, using static allocation wherever possible and optimizing for the ESP32 platform.

## Features

- **Complete Protocol Implementation**: Full implementation of the HLK-LD2450 radar protocol
- **Multi-target Tracking**: Support for tracking up to 3 targets simultaneously
- **Single-Target Mode**: Optimized mode for tracking a single target
- **Automatic Frame Processing**: Optional background task for continuous data processing
- **Thread-Safe Operation**: Mutex protection for multi-threaded environments
- **Configurable Regions**: Support for defining up to 3 detection regions
- **Flexible Configuration**: Control over baud rate, Bluetooth settings, and other device parameters
- **Efficient Memory Usage**: Optimized for embedded systems with limited resources
- **Comprehensive Error Handling**: Detailed error reporting for debugging

## Hardware Overview

The HLK-LD2450 is a 24GHz radar sensor with the following specifications:

- **Frequency**: 24GHz-24.25GHz
- **Detection Range**: 0.2m-8m
- **Angular Coverage**: 120° horizontal, 60° vertical
- **Accuracy**: ±5cm distance, ±3° angle
- **Interface**: UART (default 256000 baud)
- **Supply Voltage**: 5V DC
- **Current Consumption**: <100mA

### Connection Diagram

```
┌─────────────┐              ┌───────────┐
│             │              │           │
│    ESP32    │              │ HLK-LD2450│
│             │              │           │
│         TX_n├──────────────┤RX         │
│             │              │           │
│         RX_n├──────────────┤TX         │
│             │              │           │
│         VCC ├──────────────┤VCC (5V)   │
│             │              │           │
│         GND ├──────────────┤GND        │
│             │              │           │
└─────────────┘              └───────────┘
```

## Installation

### Method 1: Using ESP-IDF Component Manager

Add the following to your project's `idf_component.yml` file:

```yaml
dependencies:
  ld2450:
    git: https://github.com/NieRVoid/ld2450-driver.git
    version: "~1.0.0"
```

### Method 2: Manual Installation

1. Create a `components` directory in your project if it doesn't exist:
   ```bash
   mkdir -p components
   ```

2. Clone the repository into your components directory:
   ```bash
   cd components
   git clone https://github.com/NieRVoid/ld2450-driver.git ld2450
   ```

3. Include the header file in your project:
   ```c
   #include "ld2450.h"
   ```

## API Reference

### Initialization

#### Initialize the driver

```c
esp_err_t ld2450_init(const ld2450_config_t *config);
```

Initializes the LD2450 driver with the specified configuration. The configuration structure specifies UART port, pins, baud rate, and whether to use automatic data processing.

Parameters:
- `config`: Pointer to driver configuration structure

Returns:
- `ESP_OK` on success, error code otherwise

Example:
```c
ld2450_config_t config = LD2450_DEFAULT_CONFIG();
config.uart_rx_pin = GPIO_NUM_16;
config.uart_tx_pin = GPIO_NUM_17;
esp_err_t ret = ld2450_init(&config);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
}
```

#### Deinitialize the driver

```c
esp_err_t ld2450_deinit(void);
```

Deinitializes the LD2450 driver and releases all resources.

Returns:
- `ESP_OK` on success, error code otherwise

### Data Reception

#### Register a callback function for target data

```c
esp_err_t ld2450_register_target_callback(ld2450_target_cb_t callback, void *user_ctx);
```

Registers a callback function that is called whenever new target data is available.

Parameters:
- `callback`: Function pointer to the callback
- `user_ctx`: User context pointer passed to the callback function

Returns:
- `ESP_OK` on success, error code otherwise

Example:
```c
void target_callback(const ld2450_frame_t *frame, void *ctx) {
    ESP_LOGI(TAG, "Received frame with %d targets", frame->count);
    
    for (int i = 0; i < frame->count; i++) {
        const ld2450_target_t *target = &frame->targets[i];
        ESP_LOGI(TAG, "Target %d: x=%d, y=%d, distance=%.2f, angle=%.2f", 
                 i, target->x, target->y, target->distance, target->angle);
    }
}

esp_err_t ret = ld2450_register_target_callback(target_callback, NULL);
```

#### Process a frame manually

```c
esp_err_t ld2450_process_frame(const uint8_t *data, size_t length, ld2450_frame_t *frame);
```

Processes a raw radar data frame manually (useful when `auto_processing` is disabled).

Parameters:
- `data`: Raw frame data buffer
- `length`: Length of the data buffer
- `frame`: Pointer to frame structure to store parsed results

Returns:
- `ESP_OK` on success, error code otherwise

### Configuration Commands

#### Get firmware version

```c
esp_err_t ld2450_get_firmware_version(ld2450_firmware_version_t *version);
```

Retrieves the firmware version from the radar module.

Parameters:
- `version`: Pointer to structure to store version information

Returns:
- `ESP_OK` on success, error code otherwise

Example:
```c
ld2450_firmware_version_t version;
esp_err_t ret = ld2450_get_firmware_version(&version);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Firmware version: %s", version.version_string);
}
```

#### Set tracking mode

```c
esp_err_t ld2450_set_tracking_mode(ld2450_tracking_mode_t mode);
```

Sets the target tracking mode (single target or multi-target).

Parameters:
- `mode`: Tracking mode to set (`LD2450_MODE_SINGLE_TARGET` or `LD2450_MODE_MULTI_TARGET`)

Returns:
- `ESP_OK` on success, error code otherwise

#### Get tracking mode

```c
esp_err_t ld2450_get_tracking_mode(ld2450_tracking_mode_t *mode);
```

Gets the current target tracking mode.

Parameters:
- `mode`: Pointer to store the current tracking mode

Returns:
- `ESP_OK` on success, error code otherwise

#### Set serial port baud rate

```c
esp_err_t ld2450_set_baud_rate(ld2450_baud_rate_t baud_rate);
```

Sets the serial port baud rate. This setting is saved and takes effect after module restart.

Parameters:
- `baud_rate`: Baud rate to set (enum value)

Returns:
- `ESP_OK` on success, error code otherwise

#### Restore factory settings

```c
esp_err_t ld2450_restore_factory_settings(void);
```

Restores factory default settings. This takes effect after module restart.

Returns:
- `ESP_OK` on success, error code otherwise

#### Restart module

```c
esp_err_t ld2450_restart_module(void);
```

Restarts the radar module.

Returns:
- `ESP_OK` on success, error code otherwise

#### Set Bluetooth functionality

```c
esp_err_t ld2450_set_bluetooth(bool enable);
```

Enables or disables Bluetooth functionality. This setting is persistent after power-off and takes effect after restart.

Parameters:
- `enable`: true to enable Bluetooth, false to disable

Returns:
- `ESP_OK` on success, error code otherwise

#### Get MAC address

```c
esp_err_t ld2450_get_mac_address(uint8_t mac[6]);
```

Gets the module's MAC address.

Parameters:
- `mac`: Buffer to store the 6-byte MAC address

Returns:
- `ESP_OK` on success, error code otherwise

Example:
```c
uint8_t mac[6];
esp_err_t ret = ld2450_get_mac_address(mac);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
```

### Target and Region Handling

#### Set region filter

```c
esp_err_t ld2450_set_region_filter(ld2450_filter_type_t type, const ld2450_region_t regions[3]);
```

Configures region filtering.

Parameters:
- `type`: Filtering type (disabled, include only, exclude)
- `regions`: Array of 3 region definitions

Returns:
- `ESP_OK` on success, error code otherwise

#### Get region filter

```c
esp_err_t ld2450_get_region_filter(ld2450_filter_type_t *type, ld2450_region_t regions[3]);
```

Queries current region filtering configuration.

Parameters:
- `type`: Pointer to store the filtering type
- `regions`: Array of 3 region definitions to store the current configuration

Returns:
- `ESP_OK` on success, error code otherwise

## Usage Examples

### Basic Initialization

```c
#include "ld2450.h"
#include "esp_log.h"

static const char *TAG = "LD2450_EXAMPLE";

void app_main(void)
{
    // Initialize the LD2450 driver with default config
    ld2450_config_t config = LD2450_DEFAULT_CONFIG();
    
    // Customize configuration if needed
    config.uart_port = UART_NUM_2;
    config.uart_rx_pin = GPIO_NUM_16;
    config.uart_tx_pin = GPIO_NUM_17;
    config.auto_processing = true;
    
    esp_err_t ret = ld2450_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2450 driver: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 driver initialized successfully");
    
    // Get and display firmware version
    ld2450_firmware_version_t version;
    ret = ld2450_get_firmware_version(&version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Firmware version: %s", version.version_string);
    }
}
```

### Target Detection

```c
void target_callback(const ld2450_frame_t *frame, void *ctx) {
    static int count = 0;
    
    // Print every 10th frame to reduce log volume
    if (count++ % 10 == 0) {
        ESP_LOGI(TAG, "Frame with %d targets", frame->count);
        
        for (int i = 0; i < frame->count; i++) {
            const ld2450_target_t *target = &frame->targets[i];
            ESP_LOGI(TAG, "Target %d: x=%d mm, y=%d mm, dist=%.2f mm, angle=%.2f°, speed=%d cm/s", 
                     i, target->x, target->y, target->distance, target->angle, target->speed);
        }
    }
}

void app_main(void)
{
    // Initialize driver (code from previous example)
    // ...
    
    // Register callback for target detection
    ret = ld2450_register_target_callback(target_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register target callback: %s", esp_err_to_name(ret));
        return;
    }
    
    // The rest of your application code
    // ...
}
```

### Region Filtering

```c
void app_main(void)
{
    // Initialize driver (code from previous example)
    // ...
    
    // Define three detection regions
    ld2450_region_t regions[3] = {
        // Region 1: A rectangle from (0,0) to (2000,2000) mm
        {
            .x1 = 0,
            .y1 = 0,
            .x2 = 2000,
            .y2 = 2000
        },
        // Region 2: A rectangle from (-1000,0) to (0,2000) mm
        {
            .x1 = -1000,
            .y1 = 0,
            .x2 = 0,
            .y2 = 2000
        },
        // Region 3: Not used (all zeros)
        {0, 0, 0, 0}
    };
    
    // Set region filtering to only detect targets within the defined regions
    ret = ld2450_set_region_filter(LD2450_FILTER_INCLUDE_ONLY, regions);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set region filter: %s", esp_err_to_name(ret));
    }
}
```

## Data Structures

### Driver Configuration

```c
typedef struct {
    uart_port_t uart_port;      // UART port number
    int uart_rx_pin;            // GPIO pin for UART RX
    int uart_tx_pin;            // GPIO pin for UART TX
    uint32_t uart_baud_rate;    // UART baud rate
    bool auto_processing;       // Enable automatic frame processing
    int task_priority;          // Priority for auto processing task
} ld2450_config_t;
```

Default configuration:
```c
#define LD2450_DEFAULT_CONFIG() { \
    .uart_port = UART_NUM_2, \
    .uart_rx_pin = 16, \
    .uart_tx_pin = 17, \
    .uart_baud_rate = 256000, \
    .auto_processing = true, \
    .task_priority = 5, \
}
```

### Target Information

```c
typedef struct {
    int16_t x;                // X coordinate (mm)
    int16_t y;                // Y coordinate (mm)
    int16_t speed;            // Speed (cm/s)
    uint16_t resolution;      // Distance resolution (mm)
    float distance;           // Calculated distance (mm)
    float angle;              // Calculated angle in degrees
    bool valid;               // Target validity flag
} ld2450_target_t;
```

### Data Frame

```c
typedef struct {
    ld2450_target_t targets[3];  // Data for up to 3 targets
    uint8_t count;               // Number of valid targets (0-3)
    int64_t timestamp;           // ESP timestamp when data was received
} ld2450_frame_t;
```

### Firmware Version

```c
typedef struct {
    uint16_t main_version;     // Main version number
    uint32_t sub_version;      // Sub-version number
    char version_string[32];   // Formatted version string (e.g., "V1.02.22062416")
} ld2450_firmware_version_t;
```

### Region Definition

```c
typedef struct {
    int16_t x1;   // X coordinate of first corner (mm)
    int16_t y1;   // Y coordinate of first corner (mm)
    int16_t x2;   // X coordinate of diagonal corner (mm)
    int16_t y2;   // Y coordinate of diagonal corner (mm)
} ld2450_region_t;
```

### Enumerations

#### Target Tracking Mode

```c
typedef enum {
    LD2450_MODE_SINGLE_TARGET = 0x0001, // Track a single target
    LD2450_MODE_MULTI_TARGET = 0x0002   // Track multiple targets (default)
} ld2450_tracking_mode_t;
```

#### Baud Rate Options

```c
typedef enum {
    LD2450_BAUD_9600   = 0x0001, // 9600 baud
    LD2450_BAUD_19200  = 0x0002, // 19200 baud
    LD2450_BAUD_38400  = 0x0003, // 38400 baud
    LD2450_BAUD_57600  = 0x0004, // 57600 baud
    LD2450_BAUD_115200 = 0x0005, // 115200 baud
    LD2450_BAUD_230400 = 0x0006, // 230400 baud
    LD2450_BAUD_256000 = 0x0007, // 256000 baud (default)
    LD2450_BAUD_460800 = 0x0008  // 460800 baud
} ld2450_baud_rate_t;
```

#### Region Filtering Types

```c
typedef enum {
    LD2450_FILTER_DISABLED = 0x0000,   // Disable region filtering
    LD2450_FILTER_INCLUDE_ONLY = 0x0001, // Only detect targets within specified regions
    LD2450_FILTER_EXCLUDE = 0x0002      // Do not detect targets within specified regions
} ld2450_filter_type_t;
```

## Troubleshooting

### Common Issues

#### No targets detected

1. **Check physical connections**: Ensure the radar is properly connected to your ESP32 and receiving power.
2. **Verify UART pins**: Double-check that the UART pins match your hardware configuration.
3. **Verify baud rate**: The default baud rate for the HLK-LD2450 is 256000. If you've changed it, make sure your configuration matches.
4. **Check target distance**: The radar has a minimum detection distance of about 0.2m.
5. **Check region filtering**: If region filtering is enabled, make sure your regions are properly defined.

#### Communication errors

1. **Check error logs**: Look for UART errors in the ESP-IDF logs.
2. **Check voltage levels**: Make sure the radar is receiving the correct voltage (5V).
3. **Try restoring factory settings**: Use the `ld2450_restore_factory_settings()` function.
4. **Check cables**: Ensure UART cables are not too long, which can cause signal degradation.

### Debug Logging

The driver uses ESP-IDF's logging system with the tag "LD2450". You can enable debug logs by setting the log level:

```c
esp_log_level_set("LD2450", ESP_LOG_DEBUG);
```

For verbose logging (including frame data):

```c
esp_log_level_set("LD2450", ESP_LOG_VERBOSE);
```

### Technical Support

For technical support:
1. Check for firmware updates for the HLK-LD2450
2. Review the HLK-LD2450 datasheet for hardware specifications
3. Open an issue on the GitHub repository with a detailed description of the problem, including logs and hardware setup

## Technical Details

### Protocol Overview

The HLK-LD2450 uses a binary protocol over UART with distinct frame formats for data and configuration:

#### Data Frame Format

```
+--------+--------+--------+--------+----------------+--------+--------+
| Header (4 bytes)| Target Data (24 bytes)           | Footer (2 bytes)|
+--------+--------+--------+--------+----------------+--------+--------+
| 0xAA   | 0xFF   | 0x03   | 0x00   | [Target Data]  | 0x55   | 0xCC   |
+--------+--------+--------+--------+----------------+--------+--------+
```

Each target data segment is 8 bytes, supporting up to 3 targets per frame.

#### Configuration Frame Format

```
+--------+--------+--------+--------+--------+--------+--------+--------+----------------------+--------+--------+--------+--------+
| Header (4 bytes)         | Data Len (2 bytes)| CMD (2 bytes)  | Payload (variable)       | Footer (4 bytes)         |
+--------+--------+--------+--------+--------+--------+--------+--------+----------------------+--------+--------+--------+--------+
| 0xFD   | 0xFC   | 0xFB   | 0xFA   | [Len]          | [CMD]          | [Payload]             | 0x04   | 0x03   | 0x02   | 0x01   |
+--------+--------+--------+--------+--------+--------+--------+--------+----------------------+--------+--------+--------+--------+
```

### Memory Usage

The driver is optimized for efficient memory usage:
- Static allocation for frame buffers
- No dynamic memory allocation during normal operation
- Configurable task stack size (default: 4KB)
- Small UART buffer (default: 512 bytes)

### Coordinate System

The HLK-LD2450 uses a Cartesian coordinate system with the origin at the radar:

```
       Y+
       ^
       |
       |
       +----> X+
      /
     /
    Z+ (depth)
```

- X-axis: Horizontal position (negative = left, positive = right)
- Y-axis: Distance from radar (always positive)
- Angle: Calculated as -atan2(x, y) * (180/π), giving angles from -90° to +90°

## License

The HLK-LD2450 driver is provided under the MIT License, which permits use, modification, and distribution with minimal restrictions:

```
MIT License

Copyright (c) 2025 NieRVoid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

This driver is not affiliated with Hi-Link Technology Co., Ltd, the manufacturer of the HLK-LD2450 module. All trademarks are the property of their respective owners.

## Changelog

### v0.1.0 (2025-03-12)
- Initial release
- Full support for HLK-LD2450 protocol
- Support for multi-target and single-target modes
- Region filtering capabilities
- Comprehensive configuration API

## Future Development

Planned features for future releases:
- Enhanced Bluetooth configuration
- Direct GPIO triggering for motion events
- Advanced filtering options
- Power management integration
- Simple visualization tools

Contributions and feature requests are welcome through the project's GitHub repository.