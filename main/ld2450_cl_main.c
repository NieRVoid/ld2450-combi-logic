/**
 * @file main.c
 * @brief Test application for occupancy manager component
 * 
 * This application demonstrates the use of the occupancy manager with
 * multiple data sources of different reliability levels:
 * - Radar sensor (LD2450) - low reliability, continuous updates
 * - Physical button (BOOT on GPIO0) - high reliability, triggered updates
 * - Remote control (simulated via console) - medium reliability, triggered updates
 *
 * @author NieRVoid
 * @date 2025-03-15
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "ld2450.h"
#include "people_counter.h"
#include "occupancy_manager.h"

// Constants
#define TAG "main"
#define BUTTON_PIN GPIO_NUM_0      // BOOT button
#define BUTTON_DEBOUNCE_MS 300     // Debounce time in ms

// Source IDs for the occupancy manager
static occupancy_source_id_t radar_source_id;
static occupancy_source_id_t button_source_id;
static occupancy_source_id_t remote_source_id;

// State variables
static volatile uint32_t last_button_time = 0;
static SemaphoreHandle_t console_mutex;

// Forward declarations
static void register_console_commands(void);

/**
 * @brief Callback for occupancy status changes
 */
static void occupancy_status_changed(const occupancy_status_t *status, void *user_ctx) {
    // Get source name for logging
    const char *source_name = "unknown";
    if (status->source == radar_source_id) {
        source_name = "radar";
    } else if (status->source == button_source_id) {
        source_name = "button";
    } else if (status->source == remote_source_id) {
        source_name = "remote";
    }
    
    // Log the status change with detailed information
    ESP_LOGI(TAG, "===== OCCUPANCY STATUS CHANGED =====");
    ESP_LOGI(TAG, "Room is %s", status->is_occupied ? "OCCUPIED" : "UNOCCUPIED");
    ESP_LOGI(TAG, "People count: %d (%s)", status->count, 
             status->is_count_certain ? "certain" : "uncertain");
    ESP_LOGI(TAG, "Determining source: %s (reliability: %d)", 
             source_name, status->determining_reliability);
    ESP_LOGI(TAG, "====================================");
    
    // Here you would control the power supply based on status->is_occupied
}

/**
 * @brief Callback function for the people counter
 */
static void people_count_changed(int count, int entries, int exits, void* context) {
    ESP_LOGI(TAG, "People counter: count=%d, entries=%d, exits=%d", count, entries, exits);
    
    // Update the occupancy manager with the radar data
    esp_err_t err = occupancy_manager_update_count(radar_source_id, count, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update occupancy manager with radar data: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Callback function for radar target data
 */
static void radar_target_callback(const ld2450_frame_t *frame, void *user_ctx) {
    // Log target data (only valid targets)
    for (int i = 0; i < frame->count; i++) {
        const ld2450_target_t *target = &frame->targets[i];
        if (target->valid) {
            ESP_LOGD(TAG, "Radar target %d: x=%d y=%d distance=%.2f angle=%.1f", 
                    i, target->x, target->y, target->distance, target->angle);
        }
    }
    
    // The people_counter component will handle the radar data and call our callback
}

/**
 * @brief GPIO interrupt handler for button press
 */
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    
    // Simple debounce
    if ((now - last_button_time) > BUTTON_DEBOUNCE_MS) {
        last_button_time = now;
        
        // Notify the button press (processed outside of ISR)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(*(SemaphoreHandle_t*)arg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

/**
 * @brief Button handling task
 */
static void button_task(void* arg) {
    SemaphoreHandle_t button_sem = (SemaphoreHandle_t)arg;
    
    while (1) {
        // Wait for button press
        if (xSemaphoreTake(button_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Button pressed!");
            
            // Trigger presence via the button source (indicating at least 1 person)
            esp_err_t err = occupancy_manager_trigger_presence(button_source_id, 1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update occupancy manager with button press: %s", esp_err_to_name(err));
            }
        }
    }
}

/**
 * @brief Command to set room state via remote control
 */
static struct {
    struct arg_int *count;
    struct arg_lit *empty;
    struct arg_end *end;
} remote_args;

static int cmd_remote(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&remote_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, remote_args.end, argv[0]);
        return 1;
    }

    xSemaphoreTake(console_mutex, portMAX_DELAY);
    
    if (remote_args.empty->count > 0) {
        // Set empty room
        ESP_LOGI(TAG, "Remote control: Room is empty");
        occupancy_manager_update_count(remote_source_id, 0, true);
    } else if (remote_args.count->count > 0) {
        // Set specific count
        int count = remote_args.count->ival[0];
        ESP_LOGI(TAG, "Remote control: Set count to %d", count);
        occupancy_manager_update_count(remote_source_id, count, true);
    } else {
        // Default: trigger presence without specific count
        ESP_LOGI(TAG, "Remote control: Someone is present");
        occupancy_manager_trigger_presence(remote_source_id, 1);
    }
    
    xSemaphoreGive(console_mutex);
    return 0;
}

/**
 * @brief Command to get current occupancy status
 */
static int cmd_status(int argc, char **argv) {
    xSemaphoreTake(console_mutex, portMAX_DELAY);
    
    occupancy_status_t status;
    esp_err_t err = occupancy_manager_get_status(&status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get occupancy status: %s", esp_err_to_name(err));
        xSemaphoreGive(console_mutex);
        return 1;
    }
    
    // Get source name
    const char *source_name = "unknown";
    if (status.source == radar_source_id) {
        source_name = "radar";
    } else if (status.source == button_source_id) {
        source_name = "button";
    } else if (status.source == remote_source_id) {
        source_name = "remote";
    }
    
    printf("\n===== OCCUPANCY STATUS =====\n");
    printf("Room is %s\n", status.is_occupied ? "OCCUPIED" : "UNOCCUPIED");
    printf("People count: %d (%s)\n", status.count, 
           status.is_count_certain ? "certain" : "uncertain");
    printf("Determining source: %s (reliability: %d)\n", 
           source_name, status.determining_reliability);
    printf("============================\n\n");
    
    xSemaphoreGive(console_mutex);
    return 0;
}

/**
 * @brief Command to enable/disable a source
 */
static struct {
    struct arg_str *source;
    struct arg_lit *enable;
    struct arg_lit *disable;
    struct arg_end *end;
} source_args;

static int cmd_source(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&source_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, source_args.end, argv[0]);
        return 1;
    }

    xSemaphoreTake(console_mutex, portMAX_DELAY);
    
    // Validate arguments
    if (source_args.source->count == 0) {
        printf("Error: Source name required (radar, button, or remote)\n");
        xSemaphoreGive(console_mutex);
        return 1;
    }
    
    if (source_args.enable->count == 0 && source_args.disable->count == 0) {
        printf("Error: Specify either --enable or --disable\n");
        xSemaphoreGive(console_mutex);
        return 1;
    }
    
    // Determine which source to modify
    occupancy_source_id_t source_id;
    const char *source_name = source_args.source->sval[0];
    
    if (strcmp(source_name, "radar") == 0) {
        source_id = radar_source_id;
    } else if (strcmp(source_name, "button") == 0) {
        source_id = button_source_id;
    } else if (strcmp(source_name, "remote") == 0) {
        source_id = remote_source_id;
    } else {
        printf("Error: Unknown source '%s'. Use 'radar', 'button', or 'remote'\n", source_name);
        xSemaphoreGive(console_mutex);
        return 1;
    }
    
    // Set source state
    bool active = source_args.enable->count > 0;
    esp_err_t err = occupancy_manager_set_source_active(source_id, active);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set source state: %s", esp_err_to_name(err));
        printf("Error setting source state: %s\n", esp_err_to_name(err));
        xSemaphoreGive(console_mutex);
        return 1;
    }
    
    printf("Source '%s' is now %s\n", source_name, active ? "enabled" : "disabled");
    xSemaphoreGive(console_mutex);
    return 0;
}

/**
 * @brief Register commands with the console
 */
static void register_console_commands(void) {
    // Remote command
    remote_args.count = arg_int0("c", "count", "<count>", "Set specific people count");
    remote_args.empty = arg_lit0("e", "empty", "Set the room as empty");
    remote_args.end = arg_end(2);
    
    const esp_console_cmd_t cmd_remote_def = {
        .command = "remote",
        .help = "Simulate remote control input",
        .hint = "[--count <n> | --empty]",
        .func = &cmd_remote,
        .argtable = &remote_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_remote_def));
    
    // Status command
    const esp_console_cmd_t cmd_status_def = {
        .command = "status",
        .help = "Show current occupancy status",
        .hint = NULL,
        .func = &cmd_status,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_status_def));
    
    // Source command
    source_args.source = arg_str1(NULL, NULL, "<source>", "Source name (radar, button, remote)");
    source_args.enable = arg_lit0("e", "enable", "Enable the source");
    source_args.disable = arg_lit0("d", "disable", "Disable the source");
    source_args.end = arg_end(2);
    
    const esp_console_cmd_t cmd_source_def = {
        .command = "source",
        .help = "Enable or disable a data source",
        .hint = "<source> [--enable | --disable]",
        .func = &cmd_source,
        .argtable = &source_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_source_def));
}

/**
 * @brief Initialize console for command input
 */
static void initialize_console(void) {
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Configure VFS & UART so we can use std::cout/cin */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver for console I/O
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 
                                       256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    
    // Tell VFS to use UART driver
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    linenoiseSetMultiLine(1);
    linenoiseHistorySetMaxLen(10);

    /* Set command history */
    const char *history_path = ".command_history";
    linenoiseHistoryLoad(history_path);

    /* Register commands */
    esp_console_register_help_command();
    register_console_commands();
}

/**
 * @brief Console task
 */
static void console_task(void *arg) {
    initialize_console();
    
    const char *prompt = "occupancy> ";
    printf("\n"
           "Occupancy Manager Test\n"
           "Type 'help' to see available commands\n"
           "\n");

    /* Main loop */
    while (true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed. */
        char *line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }

        /* Add the command to the history if not empty */
        linenoiseHistoryAdd(line);
        linenoiseHistorySave(".command_history");

        /* Process the command - updated for ESP-IDF v5.4 */
        esp_err_t err = esp_console_run(line, NULL);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unknown command: %s\n", line);
        } else if (err != ESP_OK) {
            printf("Error: %s\n", esp_err_to_name(err));
        }

        /* linenoise allocates line buffer, we free it */
        linenoiseFree(line);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting occupancy manager test application");
    
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // Create synchronization primitives
    console_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t button_sem = xSemaphoreCreateBinary();
    
    // Initialize LD2450 radar driver
    ld2450_config_t radar_config = LD2450_DEFAULT_CONFIG();
    radar_config.uart_rx_pin = 16;
    radar_config.uart_tx_pin = 17;
    radar_config.uart_baud_rate = 256000;
    ESP_ERROR_CHECK(ld2450_init(&radar_config));
    ESP_ERROR_CHECK(ld2450_register_target_callback(radar_target_callback, NULL));
    
    // Initialize people counter
    people_counter_config_t counter_config = PEOPLE_COUNTER_DEFAULT_CONFIG();
    counter_config.vector_threshold = 1000;          // 100cm movement to count as entry/exit
    counter_config.empty_target_threshold = 5;      // 5 empty frames to consider target gone
    counter_config.detection_min_x = -2000;         // Detection area: 4m wide, 2m deep
    counter_config.detection_max_x = 2000;
    counter_config.detection_min_y = 0;
    counter_config.detection_max_y = 2000;
    counter_config.vector_threshold = 1000;          // 100cm movement to count as entry/exit
    counter_config.empty_target_threshold = 5;      // 5 empty frames to consider target gone
    counter_config.count_changed_cb = people_count_changed;
    ESP_ERROR_CHECK(people_counter_init(&counter_config));
    
    // Initialize occupancy manager
    ESP_ERROR_CHECK(occupancy_manager_init());
    ESP_ERROR_CHECK(occupancy_manager_register_callback(occupancy_status_changed, NULL));
    
    // Register sources with occupancy manager
    ESP_LOGI(TAG, "Registering data sources");
    
    // 1. Register radar (continuous, low reliability, with timeout)
    ESP_ERROR_CHECK(occupancy_manager_register_source("radar", OCCUPANCY_RELIABILITY_LOW,
                                                     OCCUPANCY_SOURCE_CONTINUOUS,
                                                     5000, // 5 second timeout
                                                     -1, &radar_source_id));
    
    // 2. Register button (triggered, high reliability, no timeout)
    ESP_ERROR_CHECK(occupancy_manager_register_source("button", OCCUPANCY_RELIABILITY_HIGH,
                                                     OCCUPANCY_SOURCE_TRIGGERED,
                                                     0, // No timeout
                                                     -1, &button_source_id));
    
    // 3. Register remote control (triggered, medium reliability, long timeout)
    ESP_ERROR_CHECK(occupancy_manager_register_source("remote", OCCUPANCY_RELIABILITY_MEDIUM,
                                                     OCCUPANCY_SOURCE_TRIGGERED,
                                                     300000, // 5 minute timeout
                                                     -1, &remote_source_id));
    
    // Configure button GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_up_en = true,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // Using priority 0 instead of a flag
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, &button_sem));
    
    // Start button task
    xTaskCreate(button_task, "button_task", 2048, button_sem, 10, NULL);
    
    // Start console task
    xTaskCreate(console_task, "console_task", 4096, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "Initialization complete - system running");
}