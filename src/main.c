#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "i2c_lcd.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define Buttons and Encoder
#define BUTTON_2_GPIO 19
#define BUTTON_3_GPIO 21
#define ENCODER_CLK_GPIO 27
#define ENCODER_DT_GPIO 33
#define ENCODER_SW_GPIO 25
#define LED_INDICATOR_GPIO 26

// Define I2C Pins
#define I2C_SDA_GPIO 23
#define I2C_SCL_GPIO 22

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

// Global state for weights
float weight_v = 1.0f;
float weight_t = 0.0f;
volatile int encoder_value = 0;
int last_encoder_value = 0;
volatile uint8_t last_clk_state = 0;

// NVS tracking
bool weights_need_saving = false;
uint32_t last_knob_turn_time = 0;

// Interrupt Service Routine for the Encoder
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    uint8_t clk = gpio_get_level(ENCODER_CLK_GPIO);
    uint8_t dt = gpio_get_level(ENCODER_DT_GPIO);
    
    if (clk != last_clk_state) {
        if (dt != clk) {
            encoder_value++;
        } else {
            encoder_value--;
        }
        last_clk_state = clk;
    }
}

// Initialize I2C Master
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}



// Initialize NVS
void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// ADC Handle and Init
adc_oneshot_unit_handle_t adc2_handle;

void sensor_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc2_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    // Pin 13 is ADC2_CHANNEL_4
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config));
}

// Read normalized voltage (0.0 to 1.0)
float get_normalized_voltage(void) {
    int adc_raw = 0;
    esp_err_t err = adc_oneshot_read(adc2_handle, ADC_CHANNEL_4, &adc_raw);
    if (err != ESP_OK) return 0.0f;
    return (float)adc_raw / 4095.0f;
}

// Calculate the final reward based on inverted tilt and weights
float calculate_reward(int tilt_index) {
    float v_norm = get_normalized_voltage();
    // Inverted tilt: 0 is looking up (reward 1.0), 11 is looking down (reward 0.0)
    float t_norm = (11.0f - (float)tilt_index) / 11.0f;
    return (weight_v * v_norm) + (weight_t * t_norm);
}

// Q-Learning Global State
float q_table[12][12][4]; 
float rewards_matrix[12][12];

// Save Q-table to NVS
void save_q_table(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(my_handle, "q_table", q_table, sizeof(q_table));
        if (err == ESP_OK) {
            nvs_commit(my_handle);
            printf("\nQ-table saved to NVS successfully.\n");
        } else {
            printf("\nFailed to save Q-table to NVS (%d).\n", err);
        }
        nvs_close(my_handle);
    }
}

// Load Q-table from NVS
void load_q_table(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(q_table);
        err = nvs_get_blob(my_handle, "q_table", q_table, &required_size);
        if (err == ESP_OK && required_size == sizeof(q_table)) {
            printf("Loaded Q-table from NVS! Turning on LED indicator.\n");
            gpio_set_level(LED_INDICATOR_GPIO, 1);
        } else {
            printf("No valid Q-table found in NVS. It will remain 0.\n");
            gpio_set_level(LED_INDICATOR_GPIO, 0);
        }
        nvs_close(my_handle);
    }
}

// Training Task
void training_task(void *pvParameters) {
    printf("\n--- STARTING TRAINING ---\n");
    gpio_set_level(LED_INDICATOR_GPIO, 0); // Turn off LED
    memset(q_table, 0, sizeof(q_table));
    memset(rewards_matrix, 0, sizeof(rewards_matrix));
    
    // 1. Grid Scan
    for (int pan = 0; pan < 12; pan++) {
        for (int tilt = 0; tilt < 12; tilt++) {
            // Move hardware
            set_servo_angle_smooth(PAN_SERVO_INDEX, pan * 15.0f);
            set_servo_angle_smooth(TILT_SERVO_INDEX, tilt * 15.0f);
            
            // Wait for motion to complete
            while (servos[PAN_SERVO_INDEX].is_moving || servos[TILT_SERVO_INDEX].is_moving) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            // Extra settle time
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Calculate and store reward
            rewards_matrix[pan][tilt] = calculate_reward(tilt);
        }
    }
    
    printf("\nGrid scan complete. Running Value Iteration...\n");
    
    // 2. Value Iteration (in memory)
    float alpha = 0.1f;
    float gamma = 0.9f;
    
    for (int iter = 0; iter < 1000; iter++) {
        for (int pan = 0; pan < 12; pan++) {
            for (int tilt = 0; tilt < 12; tilt++) {
                for (int a = 0; a < 4; a++) { // 0=PAN_LEFT, 1=PAN_RIGHT, 2=TILT_UP, 3=TILT_DOWN
                    // Determine next state
                    int next_pan = pan;
                    int next_tilt = tilt;
                    if (a == 0 && pan > 0) next_pan--; // PAN_LEFT
                    if (a == 1 && pan < 11) next_pan++; // PAN_RIGHT
                    if (a == 2 && tilt > 0) next_tilt--; // TILT_UP (0 is up)
                    if (a == 3 && tilt < 11) next_tilt++; // TILT_DOWN (11 is down)
                    
                    // Find max Q in next state
                    float max_q_next = 0.0f;
                    for (int next_a = 0; next_a < 4; next_a++) {
                        if (q_table[next_pan][next_tilt][next_a] > max_q_next) {
                            max_q_next = q_table[next_pan][next_tilt][next_a];
                        }
                    }
                    
                    // Update Q-value
                    float r = rewards_matrix[next_pan][next_tilt];
                    q_table[pan][tilt][a] = q_table[pan][tilt][a] + alpha * (r + gamma * max_q_next - q_table[pan][tilt][a]);
                }
            }
        }
    }
    
    // 3. Print Q-table Max Values
    printf("\n--- OPTIMAL POLICY (Max Q-Values) ---\n");
    for (int tilt = 0; tilt < 12; tilt++) {
        for (int pan = 0; pan < 12; pan++) {
            float max_q = 0.0f;
            for (int a = 0; a < 4; a++) {
                if (q_table[pan][tilt][a] > max_q) {
                    max_q = q_table[pan][tilt][a];
                }
            }
            printf("%.2f ", max_q);
        }
        printf("\n");
    }
    
    // 4. Return to center
    set_servo_angle_smooth(PAN_SERVO_INDEX, 90.0f);
    set_servo_angle_smooth(TILT_SERVO_INDEX, 90.0f);
    
    // 5. Save to NVS
    save_q_table();
    
    printf("\nTraining complete!\n");
    gpio_set_level(LED_INDICATOR_GPIO, 1); // Turn on LED
    
    vTaskDelete(NULL);
}

// Update the LCD display
void update_lcd_display() {
    char buf[17];
    lcd_set_cursor(0, 0);
    snprintf(buf, sizeof(buf), "wV:%.2f wT:%.2f", weight_v, weight_t);
    lcd_print(buf);
    
    // Get current physical tilt index
    int current_tilt_index = (int)(servos[TILT_SERVO_INDEX].target_angle / 15.0f + 0.5f);
    if (current_tilt_index > 11) current_tilt_index = 11;
    if (current_tilt_index < 0) current_tilt_index = 0;
    
    float v_norm = get_normalized_voltage();
    float reward = calculate_reward(current_tilt_index);
    
    lcd_set_cursor(0, 1);
    snprintf(buf, sizeof(buf), "V:%.2f R:%.2f   ", v_norm, reward);
    lcd_print(buf);
}

// Load weights from NVS
void load_weights(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        uint32_t w_t_bin = 0;
        err = nvs_get_u32(my_handle, "weight_t", &w_t_bin);
        if (err == ESP_OK) {
            memcpy(&weight_t, &w_t_bin, sizeof(float));
            weight_v = 1.0f - weight_t;
            printf("Loaded weights from NVS: w_v=%.2f, w_t=%.2f\n", weight_v, weight_t);
        }
        nvs_close(my_handle);
    } else {
        printf("NVS weights not found. Using defaults.\n");
    }
}

// Save weights to NVS
void save_weights(float t) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        uint32_t w_t_bin = 0;
        memcpy(&w_t_bin, &t, sizeof(float));
        nvs_set_u32(my_handle, "weight_t", w_t_bin);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        printf("\nWeights saved to NVS: w_t=%.2f\n", t);
    }
}

// Encoder UI Update Task (Runs at a relaxed pace to update LCD)
void encoder_task(void *pvParameters) {
    // 1. Configure the GPIO pins
    gpio_config_t enc_config = {
        .pin_bit_mask = (1ULL << ENCODER_CLK_GPIO) | (1ULL << ENCODER_DT_GPIO) | (1ULL << ENCODER_SW_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE // Trigger interrupt on any change to CLK
    };
    gpio_config(&enc_config);

    // 2. Install the GPIO ISR service and attach the handler to the CLK pin
    gpio_install_isr_service(0);
    last_clk_state = gpio_get_level(ENCODER_CLK_GPIO);
    gpio_isr_handler_add(ENCODER_CLK_GPIO, encoder_isr_handler, NULL);
    
    while (1) {
        // We only check if the interrupt has changed the value
        if (abs(encoder_value - last_encoder_value) >= 2) {
            if (encoder_value > last_encoder_value) {
                weight_t += 0.05f;
            } else {
                weight_t -= 0.05f;
            }
            
            if (weight_t > 1.0f) weight_t = 1.0f;
            if (weight_t < 0.0f) weight_t = 0.0f;
            
            weight_v = 1.0f - weight_t;
            last_encoder_value = encoder_value;
            
            update_lcd_display();
            
            // Mark for delayed saving
            weights_need_saving = true;
            last_knob_turn_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        
        // Execute delayed save if 3 seconds have passed
        if (weights_need_saving) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time - last_knob_turn_time >= 3000) {
                save_weights(weight_t);
                weights_need_saving = false;
            }
        }
        
        // Update LCD periodically with live sensor values (every ~500ms)
        static uint32_t last_lcd_update = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_lcd_update > 500) {
            update_lcd_display();
            last_lcd_update = current_time;
        }
        
        // Also check encoder switch
        if (gpio_get_level(ENCODER_SW_GPIO) == 0) {
            printf("\nEncoder Button Pressed! Wiping Q-table and starting training...\n");
            xTaskCreate(training_task, "training_task", 8192, NULL, 4, NULL);
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Check for UI updates at a safe 20Hz
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

    // 1. Configure Buttons (Button 2 and Button 3) and LED
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_2_GPIO) | (1ULL << BUTTON_3_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_config);
    
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_INDICATOR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);
    gpio_set_level(LED_INDICATOR_GPIO, 0); // Initially off

    // 2. Initialize NVS and Load Configuration
    init_nvs();
    load_weights();
    load_q_table();

    // 3. Initialize Sensors
    sensor_init();

    // 4. Initialize Servos
    servo_init();

    // 5. Initialize I2C and LCD
    i2c_master_init();
    lcd_init();
    update_lcd_display();

    // 6. Start Tasks
    xTaskCreate(servo_motion_task, "servo_motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_input_task, "serial_input_task", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);

    printf("\nInitialization complete. LCD should show initial weights.\n");

    // 6. Main loop to poll the buttons
    while (1) {
        if (gpio_get_level(BUTTON_3_GPIO) == 0) {
            printf("\nButton 3 pressed! Setting motors to position 0 smoothly.\n");
            set_servo_angle_smooth(PAN_SERVO_INDEX, 0.0f);
            set_servo_angle_smooth(TILT_SERVO_INDEX, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        
        if (gpio_get_level(BUTTON_2_GPIO) == 0) {
            // Generate random angles within the operational range
            float rand_pan = (float)(esp_random() % (int)(SERVO_MAX_DEGREE + 1));
            float rand_tilt = (float)(esp_random() % (int)(SERVO_MAX_DEGREE + 1));
            
            printf("\nButton 2 pressed! Randomizing motors smoothly: Pan=%.1f, Tilt=%.1f\n", rand_pan, rand_tilt);
            set_servo_angle_smooth(PAN_SERVO_INDEX, rand_pan);
            set_servo_angle_smooth(TILT_SERVO_INDEX, rand_tilt);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}