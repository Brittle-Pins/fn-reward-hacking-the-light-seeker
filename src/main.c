#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

// Define Button 3
#define BUTTON_3_GPIO 21

// Define Servos
#define SERVO_PAN_GPIO 14
#define SERVO_TILT_GPIO 32

// LEDC configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_PAN_CHANNEL        LEDC_CHANNEL_0
#define LEDC_TILT_CHANNEL       LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // 13-bit duty resolution (8192)
#define LEDC_FREQUENCY          50                // 50 Hz = 20ms period

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        180  // We'll allow up to 180 for calibration

// Helper function to convert angle in degrees to PWM duty cycle
uint32_t angle_to_duty(uint32_t angle) {
    if (angle > SERVO_MAX_DEGREE) {
        angle = SERVO_MAX_DEGREE;
    }
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;
    
    // duty = (pulse_width / 20000us) * 8192
    return (pulse_width * 8192) / 20000;
}

// Task for handling Serial Monitor Input
void serial_input_task(void *pvParameters) {
    char input_buf[64];
    
    while (1) {
        printf("\nEnter pan angle (0-180): ");
        fflush(stdout);

        // Read a line from standard input
        if (fgets(input_buf, sizeof(input_buf), stdin) != NULL) {
            // Remove newline character
            input_buf[strcspn(input_buf, "\r\n")] = 0;
            
            if (strlen(input_buf) > 0) {
                int angle = 0;
                if (sscanf(input_buf, "%d", &angle) == 1) {
                    if (angle >= 0 && angle <= 180) {
                        printf("\nMoving Pan Servo to %d degrees.\n", angle);
                        uint32_t duty = angle_to_duty(angle);
                        ledc_set_duty(LEDC_MODE, LEDC_PAN_CHANNEL, duty);
                        ledc_update_duty(LEDC_MODE, LEDC_PAN_CHANNEL);
                    } else {
                        printf("\nError: Angle must be between 0 and 180.\n");
                    }
                } else {
                    printf("\nError: Invalid number.\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
    }
}

void app_main(void) {
    // 0. Configure standard I/O to use the console UART
    // This allows fgets and printf to work nicely over the PlatformIO Serial Monitor
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // 1. Configure Button 3
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_3_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_config);

    // 2. Configure LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 3. Configure LEDC Channels for Pan and Tilt
    ledc_channel_config_t ledc_channel_pan = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_PAN_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PAN_GPIO,
        .duty           = angle_to_duty(90), // start at middle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_pan);

    ledc_channel_config_t ledc_channel_tilt = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_TILT_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_TILT_GPIO,
        .duty           = angle_to_duty(90), // start at middle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_tilt);

    printf("\nInitialization complete. Waiting for Button 3 (GPIO 21) press...\n");

    // Start the serial input task
    xTaskCreate(serial_input_task, "serial_input_task", 4096, NULL, 5, NULL);

    // 4. Main loop to poll the button
    while (1) {
        if (gpio_get_level(BUTTON_3_GPIO) == 0) {
            printf("\nButton 3 pressed! Setting motors to position 0.\n");
            
            uint32_t duty_0 = angle_to_duty(0);
            
            ledc_set_duty(LEDC_MODE, LEDC_PAN_CHANNEL, duty_0);
            ledc_update_duty(LEDC_MODE, LEDC_PAN_CHANNEL);
            
            ledc_set_duty(LEDC_MODE, LEDC_TILT_CHANNEL, duty_0);
            ledc_update_duty(LEDC_MODE, LEDC_TILT_CHANNEL);
            
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}