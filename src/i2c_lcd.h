#pragma once

#include <stdint.h>
#include "driver/i2c.h"

// Define I2C parameters
#define I2C_MASTER_NUM I2C_NUM_0
#define LCD_ADDR 0x3E

void lcd_init(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void lcd_clear(void);
