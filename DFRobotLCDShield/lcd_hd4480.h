/**
 * @file    lcd_hd4480.h
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
LCD_STATUS Lcd4480Init( uint8_t bLin, uint8_t bCol );

/**
 *  @brief      write a string on the display
 * 
 */
LCD_STATUS Lcd4480Write( const char* pcMsg );

uint8_t Lcd4480GetLin(void);

uint8_t Lcd4480GetCol(void);

uint8_t Lcd4480GetCurLin(void);

uint8_t Lcd4480GetCurCol(void);

LCD_STATUS Lcd4480Clear( void );

LCD_STATUS Lcd4480Home( void );

LCD_STATUS Lcd4480SetCursor( uint8_t x, uint8_t y );

LCD_STATUS Lcd4480Cursor( void );

LCD_STATUS Lcd4480NoCursor( void );

LCD_STATUS Lcd4480Blink( void );

LCD_STATUS Lcd4480NoBlink( void );

LCD_STATUS Lcd4480Display( void );

LCD_STATUS Lcd4480NoDisplay( void );

LCD_STATUS Lcd4480ScrollDisplayLeft( void );

LCD_STATUS Lcd4480ScrollDisplayRight( void );

LCD_STATUS Lcd4480ScrollAutoScroll( void );

LCD_STATUS Lcd4480ScrollNoAutoScroll( void );

LCD_STATUS Lcd4480ScrollLeftToRight( void );

LCD_STATUS Lcd4480ScrollRightToLeft( void );

LCD_STATUS Lcd4480ScrollCreateChar( void );

LCD_STATUS Lcd4480BackLightOn( void );

LCD_STATUS Lcd4480BackLightOff( void );

#endif //~ _LCD_HD4480_H_
