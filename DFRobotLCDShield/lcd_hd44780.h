/**
 * @file    lcd_hd44780.h
 * @brief   
 * @author  Rafael Dias <rdmeneze@gmail.com>
 * @date    ago/2015
 */

#ifndef _LCD_HD4480_H_
#define _LCD_HD4480_H_

#include <stdint.h>

typedef enum
{
    LCD_OK  ,
    LCD_ERROR,
} LCD_STATUS;

/**
 * @brief       initialize the LCD 4480
 * @param[in]   bLin
 * @param[in]   bCol
 * @return      LCD_OK  initialization OK
 * @return      LCD_ERROR initialization ERROR
 */
LCD_STATUS Lcd44780Init( uint8_t bLin, uint8_t bCol );

/**
 *  @brief      write a string on the display
 * 
 */
LCD_STATUS Lcd44780Write( const char* pcMsg );

uint8_t Lcd44780GetLin(void);

uint8_t Lcd44780GetCol(void);

uint8_t Lcd44780GetCurLin(void);

uint8_t Lcd44780GetCurCol(void);

LCD_STATUS Lcd44780Clear( void );

LCD_STATUS Lcd44780Home( void );

LCD_STATUS Lcd44780SetCursor( uint8_t x, uint8_t y );

LCD_STATUS Lcd44780Cursor( void );

LCD_STATUS Lcd44780NoCursor( void );

LCD_STATUS Lcd44780Blink( void );

LCD_STATUS Lcd44780NoBlink( void );

LCD_STATUS Lcd44780Display( void );

LCD_STATUS Lcd44780NoDisplay( void );

LCD_STATUS Lcd44780ScrollDisplayLeft( void );

LCD_STATUS Lcd44780ScrollDisplayRight( void );

LCD_STATUS Lcd44780ScrollAutoScroll( void );

LCD_STATUS Lcd44780ScrollNoAutoScroll( void );

LCD_STATUS Lcd44780ScrollLeftToRight( void );

LCD_STATUS Lcd44780ScrollRightToLeft( void );

LCD_STATUS Lcd44780ScrollCreateChar( void );

LCD_STATUS Lcd44780BackLightOn( void );

LCD_STATUS Lcd44780BackLightOff( void );

#endif //~ _LCD_HD4480_H_
