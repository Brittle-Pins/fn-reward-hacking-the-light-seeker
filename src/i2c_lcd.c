#include "i2c_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// For Grove 16x2 and similar native I2C LCDs (Address usually 0x3E)
// 0x80 indicates command, 0x40 indicates data

static void lcd_send_byte(uint8_t val, uint8_t is_data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    if (is_data) {
        i2c_master_write_byte(cmd, 0x40, true);
    } else {
        i2c_master_write_byte(cmd, 0x80, true);
    }
    
    i2c_master_write_byte(cmd, val, true);
    
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for LCD to power up
    
    // Function set: 8-bit, 2 lines, 5x8 dots
    lcd_send_byte(0x38, 0); 
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Display on, cursor off, blink off
    lcd_send_byte(0x0C, 0); 
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Entry mode set: increment automatically
    lcd_send_byte(0x06, 0); 
    vTaskDelay(pdMS_TO_TICKS(10));
    
    lcd_clear();
}

void lcd_clear(void) {
    lcd_send_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); // Clear takes a long time
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    lcd_send_byte(0x80 | (col + row_offsets[row]), 0);
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_send_byte(*str++, 1);
    }
}
