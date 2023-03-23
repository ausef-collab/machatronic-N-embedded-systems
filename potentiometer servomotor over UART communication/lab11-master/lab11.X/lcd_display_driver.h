/* 
 * File:   lcd_display_driver.h
 * Author: amr usef
 * Date: Created on Nov 08, 2020, 8:46 PM
 * Discripition: This file contains the function naming to be used in main.c
 */
#ifndef LCD_DISPLAY_DRIVER_H
#define	LCD_DISPLAY_DRIVER_H

void lcd_display_driver_enable();

void lcd_display_driver_initialize();

void lcd_display_driver_clear();

void lcd_display_driver_write(char * data, int length);

void display_driver_use_first_line();

void display_drive_use_second_line();

#endif	/* LCD_DISPLAY_DRIVER_H */

