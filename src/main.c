#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          50

// Servo parameters
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE        165.0 // Back to 165 as the physical limit

// Motion parameters
#define SERVO_UPDATE_RATE_HZ    50
#define SERVO_UPDATE_PERIOD_MS  (1000 / SERVO_UPDATE_RATE_HZ)
#define MOTION_FULL_RANGE_MS    3000.0 // 3 seconds for full 165 degree motion

typedef struct {
    int channel;
    float current_angle;
    float start_angle;
    float target_angle;
    uint32_t start_time_ms;
    uint32_t duration_ms;
    bool is_moving;
} servo_state_t;

// Global state for servos
servo_state_t servos[2];
#define PAN_SERVO_INDEX 0
#define TILT_SERVO_INDEX 1

// Helper function to convert angle in degrees to PWM duty cycle
uint32_t angle_to_duty(float angle) {
    if (angle > SERVO_MAX_DEGREE) {
        angle = SERVO_MAX_DEGREE;
    } else if (angle < 0.0f) {
        angle = 0.0f;
    }
    float pulse_width = SERVO_MIN_PULSEWIDTH_US + 
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;
    return (uint32_t)((pulse_width * 8192) / 20000.0f);
}

// Write the duty to the hardware
void update_servo_hw(int channel, float angle) {
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

// Initialize the servo hardware and state
void servo_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_pan = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_PAN_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PAN_GPIO,
        .duty           = angle_to_duty(90.0f), 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_pan);

    ledc_channel_config_t ledc_channel_tilt = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_TILT_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_TILT_GPIO,
        .duty           = angle_to_duty(90.0f),
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_tilt);

    // Init state
    servos[PAN_SERVO_INDEX] = (servo_state_t){LEDC_PAN_CHANNEL, 90.0f, 90.0f, 90.0f, 0, 0, false};
    servos[TILT_SERVO_INDEX] = (servo_state_t){LEDC_TILT_CHANNEL, 90.0f, 90.0f, 90.0f, 0, 0, false};
}

// API to request a smooth movement
void set_servo_angle_smooth(int servo_index, float target_angle) {
    if (servo_index < 0 || servo_index > 1) return;
    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > SERVO_MAX_DEGREE) target_angle = SERVO_MAX_DEGREE;
    
    servo_state_t *s = &servos[servo_index];
    
    // If we're already close, don't bother moving
    if (fabs(target_angle - s->current_angle) < 0.1f) return;

    s->start_angle = s->current_angle;
    s->target_angle = target_angle;
    s->start_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // Scale duration based on distance to move (3000ms for full 165 degrees)
    float distance = fabs(target_angle - s->start_angle);
    s->duration_ms = (uint32_t)((distance / SERVO_MAX_DEGREE) * MOTION_FULL_RANGE_MS);
    
    // Handle very short durations to avoid div by zero
    if (s->duration_ms < SERVO_UPDATE_PERIOD_MS) {
        s->duration_ms = SERVO_UPDATE_PERIOD_MS;
    }
    
    s->is_moving = true;
}

// The background task that actually updates the motors
void servo_motion_task(void *pvParameters) {
    while (1) {
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        
        for (int i = 0; i < 2; i++) {
            servo_state_t *s = &servos[i];
            if (s->is_moving) {
                uint32_t elapsed = now - s->start_time_ms;
                
                if (elapsed >= s->duration_ms) {
                    // Motion complete
                    s->current_angle = s->target_angle;
                    s->is_moving = false;
                    update_servo_hw(s->channel, s->current_angle);
                } else {
                    // Normalize time from 0.0 to 1.0
                    float t = (float)elapsed / (float)s->duration_ms;
                    
                    // Ease in-out using cosine
                    float ease_progress = (1.0f - cosf(M_PI * t)) / 2.0f;
                    
                    // Interpolate
                    s->current_angle = s->start_angle + (s->target_angle - s->start_angle) * ease_progress;
                    update_servo_hw(s->channel, s->current_angle);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_PERIOD_MS));
    }
}

// Task for handling Serial Monitor Input
void serial_input_task(void *pvParameters) {
    char input_buf[64];
    
    while (1) {
        printf("\nEnter command (p <angle> for Pan, t <angle> for Tilt): ");
        fflush(stdout);

        if (fgets(input_buf, sizeof(input_buf), stdin) != NULL) {
            input_buf[strcspn(input_buf, "\r\n")] = 0;
            
            if (strlen(input_buf) > 0) {
                char motor;
                float angle = 0;
                
                if (sscanf(input_buf, "%c %f", &motor, &angle) == 2) {
                    if (motor == 'p' || motor == 'P') {
                        printf("\nMoving Pan Servo to %.1f degrees smoothly.\n", angle);
                        set_servo_angle_smooth(PAN_SERVO_INDEX, angle);
                    } else if (motor == 't' || motor == 'T') {
                        printf("\nMoving Tilt Servo to %.1f degrees smoothly.\n", angle);
                        set_servo_angle_smooth(TILT_SERVO_INDEX, angle);
                    } else {
                        printf("\nError: Unknown motor. Use 'p' or 't'.\n");
                    }
                } else {
                    printf("\nError: Invalid format. Example: p 90\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void app_main(void) {
    // 0. Configure standard I/O to use the console UART
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

    // 2. Initialize Servos
    servo_init();

    // 3. Start Tasks
    xTaskCreate(servo_motion_task, "servo_motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_input_task, "serial_input_task", 4096, NULL, 5, NULL);

    printf("\nInitialization complete. Waiting for commands...\n");

    // 4. Main loop to poll the button
    while (1) {
        if (gpio_get_level(BUTTON_3_GPIO) == 0) {
            printf("\nButton 3 pressed! Setting motors to position 0 smoothly.\n");
            set_servo_angle_smooth(PAN_SERVO_INDEX, 0.0f);
            set_servo_angle_smooth(TILT_SERVO_INDEX, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}