/* 
 * File:   lcd_display_driver.c
 * Author: amr usef
 * Date: Created on Nov 01, 2020, 8:46 PM
 * Discripition: This file contains the functions implemntation to be able to read and write to the LCD screen
 */
#include <stdio.h>
#include <xc.h>
#include "lcd_display_driver.h"

//implementing lcd display driver enable 
void lcd_display_driver_enable(){
    TRISDbits.TRISD4 =0;//setting D4 as output
    LATDbits.LATD4 =1; //D4 enable bit to on
    int j;//naming a var
    for(j=0; j<=11000; j++); //delay of 15000 for enable to stay on
    LATDbits.LATD4 =0; //D4 enable to off
}

void lcd_display_driver_clear(){
    TRISBbits.TRISB15 =0;//set RS to be output 
    TRISDbits.TRISD5 =0; //reasure that R/W is output
    LATBbits.LATB15 = 0; //setting RS to 0V
    LATDbits.LATD5 = 0;  //setting RS to 0V
    LATE = 0b00000001; //set display driver clear by setting DB0 to 1
    lcd_display_driver_enable();
}
void lcd_display_driver_write(char*data, int length){
     TRISBbits.TRISB15 =0;//set RS to be output 
     LATBbits.LATB15 = 1; //setting RS to 3.3V
     int i;
     LATE = 0b00000000;// clear LATE just incase it is preset
     for(i=0; i < length; i++){
         LATE = data[i]; // translate between asc and letters to lcd 
         lcd_display_driver_enable();
     }
}

void lcd_display_driver_initialize(){
    //function set
     TRISBbits.TRISB15 =0;//set RS to be output 
     TRISDbits.TRISD5 =0; //reasure that R/W is output
     LATBbits.LATB15 = 0; //setting RS to 0V
     LATDbits.LATD5 = 0;  //setting RS to 0V
    LATE = 0b00111000;//initiat set function to have dual line
     int i;
     for(i =0; i< 1000; i++);//delay
     lcd_display_driver_enable(); //to be able to write and read
     //display on/off function
     LATE =0b00001100; //cursor off blink off display on
     for(i =0; i< 1000; i++);//delay
     lcd_display_driver_enable(); //to be able to write and read
     
     lcd_display_driver_clear();//calling clear function to clear any displays 
     for(i =0; i< 16000; i++);//delay
       
     LATE = 0b00000100; //entry mod set
     lcd_display_driver_enable(); //to be able to write and read
}