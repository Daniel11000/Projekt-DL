/******************************************************************************
 * This file is a part of the SM2 Tutorial (C).                               *                                                 *
 ******************************************************************************/

/**
 * @file lcd1602.h
 * @author Koryciak
 * @date Sep 2020
 * @brief File containing enums, structures and declarations for LCD 2x16.
 * @ver 0.1
 */

#ifndef LCD1602_H
#define LCD1602_H

#include "frdm_bsp.h"
#include "i2c.h"


/**
 * @brief LCD1602 initialization.
 */
void LCD1602_Init(void);
/**
 * @brief Print on the display.
 *
 * @param String to display.
 */
void LCD1602_Print(char *str);
/**
 * @brief Clear whole display.
 */
void LCD1602_ClearAll(void);
/**
 * @brief Set cursor on display.
 *
 * @param Column.
 * @param Row.
 */
void LCD1602_SetCursor(uint8_t col, uint8_t row);
/**
 * @brief Backlight turn on/off.
 *
 * @param State of the backlight.
 */
void LCD1602_Backlight(uint8_t state);
void LCD1602_Blink_On(void);
void LCD1602_Blink_Off(void);
void LCD1602_Blink_Off_Cursor_On(void);
void LCD1602_PL_CH(void);
void LCD1602_PrintPL(char *str);
uint8_t LCD1602_BF_AC(uint8_t *ptr);

#endif  /* LCD1602_H */
