/**
 * @file    lcd_hd4480.h
 * @brief   
 * @author  Rafael Dias <rdmeneze@gmail.com>
 * @date    ago/2015
 */

#include "lcd_hd4480.h"
#include "ttimer.h"
#include "util.h"
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the hardware doesn't reset the LCD, so we
// can't assume that its in that state

//
//   http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
//

//-----------------------------------------------------------------------------

// LCD module information 
#define LCD_DDADR_LINE1 0x00        // start of line 1 
#define LCD_DDADR_LINE2 0x40        // start of line 2 
//#define LCD_LINETHREE 0x14    // start of line 3 (20x4) 
//#define LCD_LINEFOUR 0x54     // start of line 4 (20x4) 
#define LCD_DDADR_LINE3 0x10    // start of line 3 (16x4) 
#define LCD_DDADR_LINE4 0x50     // start of line 4 (16x4)

// LCD instructions 

typedef enum
{
    LCD_CLEAR 			= 0x01, 	//	0b00000001	// replace all characters with ASCII 'space' 
    LCD_HOME 			= 0x02, 	// 	0b00000010	// return cursor to first position on first line 
    //LCD_ENTRYMODE 		= 0x06,	    // 	0b00000110	// shift cursor from left to right on read/write 
    LCD_DISPLAYOFF 		= 0x08,	    // 	0b00001000	// turn display off 
    LCD_DISPLAYON 		= 0x0c,	    //	0b00001100	// display on, cursor off, don't blink character 
    LCD_FUNCTIONRESET 	= 0x30,	    //	0b00110000 	// reset the LCD 
    LCD_FUNCTIONSET4BIT = 0x28,	    //	0b00101000 	// 4-bit data, 2-line display, 5 x 7 font 
    LCD_SETCURSOR 		= 0x80,	    //	0b10000000	// set cursor position    
    
    // Set Entry Mode ------------- 0b000001xx
    LCD_ENTRY_DECREASE  = 0x04 | 0x00,
    LCD_ENTRY_INCREASE  = 0x04 | 0x02,
    LCD_ENTRY_NOSHIFT   = 0x04 | 0x00,
    LCD_ENTRY_SHIFT     = 0x04 | 0x01,
   
    // Set Display ---------------- 0b00001xxx    
    LCD_DISPLAY_OFF     = 0x08 | 0x00,
    LCD_DISPLAY_ON      = 0x08 | 0x04,
    LCD_CURSOR_OFF      = 0x08 | 0x00,
    LCD_CURSOR_ON       = 0x08 | 0x02,
    LCD_BLINKING_OFF    = 0x08 | 0x00,
    LCD_BLINKING_ON     = 0x08 | 0x01,
    
    // Set Shift ------------------ 0b0001xxxx
    LCD_CURSOR_MOVE     = 0x10 | 0x00,
    LCD_DISPLAY_SHIFT   = 0x10 | 0x08,
    LCD_SHIFT_LEFT      = 0x10 | 0x00,
    LCD_SHIFT_RIGHT     = 0x10 | 0x04,
    
    // Set Function --------------- 0b001xxxxx
    LCD_FUNCTION_4BIT   = 0x20 | 0x00,
    LCD_FUNCTION_8BIT   = 0x20 | 0x10, 
    LCD_FUNCTION_1LINE  = 0x20 | 0x00, 
    LCD_FUNCTION_2LINE  = 0x20 | 0x08, 
    LCD_FUNCTION_5X7    = 0x20 | 0x00,
    LCD_FUNCTION_5X10   = 0x20 | 0x04,
    
    LCD_SOFT_RESET      =  0x30,
    
    // Set CG RAM Address --------- 0b01xxxxxx  (Character Generator RAM)
    LCD_SET_CGADR       = 0x40,
    
    LCD_GC_CHAR0        = 0,
    LCD_GC_CHAR1        = 1,
    LCD_GC_CHAR2        = 2,
    LCD_GC_CHAR3        = 3,
    LCD_GC_CHAR4        = 4,
    LCD_GC_CHAR5        = 5,
    LCD_GC_CHAR6        = 6,
    LCD_GC_CHAR7        = 7,
    
    // Set DD RAM Address --------- 0b1xxxxxxx  (Display Data RAM)
    LCD_SET_DDADR       = 0x80,
} LCD_CMD;

 
//-----------------------------------------------------------------------------

/* pin definitions */ 
#define SYSCTL_PERIPH_LCD_DB4   SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_DB4       GPIO_PORTE_BASE
#define GPIO_PIN_LCD_DB4        GPIO_PIN_0

#define SYSCTL_PERIPH_LCD_DB5   SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_DB5       GPIO_PORTE_BASE
#define GPIO_PIN_LCD_DB5        GPIO_PIN_1

#define SYSCTL_PERIPH_LCD_DB6   SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_DB6       GPIO_PORTE_BASE
#define GPIO_PIN_LCD_DB6        GPIO_PIN_2

#define SYSCTL_PERIPH_LCD_DB7   SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_DB7       GPIO_PORTE_BASE
#define GPIO_PIN_LCD_DB7        GPIO_PIN_3

#define SYSCTL_PERIPH_LCD_RS    SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_RS        GPIO_PORTE_BASE
#define GPIO_PIN_LCD_RS         GPIO_PIN_4

#define SYSCTL_PERIPH_LCD_EN    SYSCTL_PERIPH_GPIOE
#define GPIO_BASE_LCD_EN        GPIO_PORTE_BASE
#define GPIO_PIN_LCD_EN         GPIO_PIN_5

#define SYSCTL_PERIPH_LCD_BK    SYSCTL_PERIPH_GPIOC
#define GPIO_BASE_LCD_BK        GPIO_PORTC_BASE
#define GPIO_PIN_LCD_BK         GPIO_PIN_4

typedef enum
{
    LCD_DB4,    //! DB4 
    LCD_DB5,    //! DB5
    LCD_DB6,    //! DB6
    LCD_DB7,    //! DB7
    LCD_RS,     //! RS
    LCD_EN,     //! EN
    LCD_BK,     //! BK
} PIN_LCD;

typedef enum 
{
    CMD,
    DATA
}RS_TYPE;

//-----------------------------------------------------------------------------

/**
 *  @struct STGpioConfig
 *  @brief  struct to represent the GPIO configurations
 */
struct STGpioConfig
{
    uint32_t    dwSYSCTL;       /**< the SYSCTL value   */
    uint32_t    dwBASE;         /**< the PortBase value */
    uint32_t    dwPin;          /**< the Pin value      */
};

//-----------------------------------------------------------------------------

static uint8_t bInit = 0;

const struct STGpioConfig stGpioLcdCfg[] = 
{
    [LCD_DB4] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_DB4,
        .dwBASE   = GPIO_BASE_LCD_DB4    ,
        .dwPin    = GPIO_PIN_LCD_DB4     ,        
    },    
    [LCD_DB5] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_DB5,
        .dwBASE   = GPIO_BASE_LCD_DB5    ,
        .dwPin    = GPIO_PIN_LCD_DB5     ,
    },    
    [LCD_DB6] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_DB6,
        .dwBASE   = GPIO_BASE_LCD_DB6    ,
        .dwPin    = GPIO_PIN_LCD_DB6     ,
    },    
    [LCD_DB7] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_DB7,
        .dwBASE   = GPIO_BASE_LCD_DB7    ,
        .dwPin    = GPIO_PIN_LCD_DB7     ,
    },    
    [LCD_RS ] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_RS,
        .dwBASE   = GPIO_BASE_LCD_RS    ,
        .dwPin    = GPIO_PIN_LCD_RS     ,
    },    
    [LCD_EN ] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_EN,
        .dwBASE   = GPIO_BASE_LCD_EN    ,
        .dwPin    = GPIO_PIN_LCD_EN     ,
    },    
    [LCD_BK ] = 
    {
        .dwSYSCTL = SYSCTL_PERIPH_LCD_BK,
        .dwBASE   = GPIO_BASE_LCD_BK    ,
        .dwPin    = GPIO_PIN_LCD_BK     ,
    },    
};

//-----------------------------------------------------------------------------

void Lcd4480WriteDB7( uint8_t bData )
{
    const PIN_LCD pin = LCD_DB7;
    
    if ( bData )
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, stGpioLcdCfg[pin].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, ~stGpioLcdCfg[pin].dwPin ); 
    }    
    
    return;
}

//-----------------------------------------------------------------------------

void Lcd4480WriteDB6( uint8_t bData )
{
    const PIN_LCD pin = LCD_DB6;
    
    if ( bData )
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, stGpioLcdCfg[pin].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, ~stGpioLcdCfg[pin].dwPin ); 
    }    
}

//-----------------------------------------------------------------------------

void Lcd4480WriteDB5( uint8_t bData )
{
    const PIN_LCD pin = LCD_DB5;
    
    if ( bData )
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, stGpioLcdCfg[pin].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, ~stGpioLcdCfg[pin].dwPin ); 
    }    
}

//-----------------------------------------------------------------------------

void Lcd4480WriteDB4( uint8_t bData )
{
    const PIN_LCD pin = LCD_DB4;
    
    if ( bData )
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, stGpioLcdCfg[pin].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, ~stGpioLcdCfg[pin].dwPin ); 
    }    
}

//-----------------------------------------------------------------------------

void Lcd4480WriteEN( uint8_t bData )
{
    if ( bData )
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_EN].dwBASE,stGpioLcdCfg[LCD_EN].dwPin, stGpioLcdCfg[LCD_EN].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_EN].dwBASE,stGpioLcdCfg[LCD_EN].dwPin, ~stGpioLcdCfg[LCD_EN].dwPin ); 
    }
    
    return;
}

//-----------------------------------------------------------------------------

void Lcd4480PulseEN( void )
{
    Lcd4480WriteEN(1);
    delay(1);
    Lcd4480WriteEN(0);
}

//-----------------------------------------------------------------------------

void Lcd4480WriteRS( RS_TYPE rs )
{
    switch( rs )
    {
        case CMD:
            GPIOPinWrite( stGpioLcdCfg[LCD_RS].dwBASE,stGpioLcdCfg[LCD_RS].dwPin, ~stGpioLcdCfg[LCD_RS].dwPin );
        break;
    
        case DATA:
        default:
            GPIOPinWrite( stGpioLcdCfg[LCD_RS].dwBASE,stGpioLcdCfg[LCD_RS].dwPin, stGpioLcdCfg[LCD_RS].dwPin );
        break;
    }

    return;
}

//-----------------------------------------------------------------------------

void Lcd4480DataWrite( uint8_t bData )
{
    uint8_t i = 7;
    
    Lcd4480WriteRS( DATA );
    
    Lcd4480WriteDB7( bData & (1<<(i--)));
    Lcd4480WriteDB6( bData & (1<<(i--)));
    Lcd4480WriteDB5( bData & (1<<(i--)));
    Lcd4480WriteDB4( bData & (1<<(i--)));
    
    Lcd4480PulseEN( );
    delay( 1 );
    
    Lcd4480WriteDB7( bData & (1<<(i--)));
    Lcd4480WriteDB6( bData & (1<<(i--)));
    Lcd4480WriteDB5( bData & (1<<(i--)));
    Lcd4480WriteDB4( bData & (1<<(i--)));

    Lcd4480PulseEN( );
    
    delay( 1 );
    
    return;
}

//-----------------------------------------------------------------------------

void Lcd4480CmdWrite( uint8_t cmd )
{
    uint8_t i = 7;
    
    Lcd4480WriteRS( CMD );
  
    Lcd4480WriteDB7( cmd & (1<<(i--)));
    Lcd4480WriteDB6( cmd & (1<<(i--)));
    Lcd4480WriteDB5( cmd & (1<<(i--)));
    Lcd4480WriteDB4( cmd & (1<<(i--)));
    
    Lcd4480PulseEN( );
    delay( 1 );
    
    Lcd4480WriteDB7( cmd & (1<<(i--)));
    Lcd4480WriteDB6( cmd & (1<<(i--)));
    Lcd4480WriteDB5( cmd & (1<<(i--)));
    Lcd4480WriteDB4( cmd & (1<<(i--)));

    Lcd4480PulseEN( );    
    delay( 1 );
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Init( uint8_t bLin, uint8_t bCol )
{
    LCD_STATUS xRet = LCD_ERROR;
    uint8_t bCounter;
    
    if ( !bInit )
    {
        // configura os GPIOs 
        for ( bCounter = 0; bCounter < GET_ARRAY_LEN( stGpioLcdCfg ); bCounter++ )
        {
            SysCtlPeripheralEnable( stGpioLcdCfg[bCounter].dwSYSCTL );
            GPIOPinTypeGPIOOutput( stGpioLcdCfg[bCounter].dwBASE, stGpioLcdCfg[bCounter].dwPin );
            GPIOPinWrite( stGpioLcdCfg[bCounter].dwBASE,stGpioLcdCfg[bCounter].dwPin, ~stGpioLcdCfg[bCounter].dwPin ); 
        }
        
        delay( 100 );
        
        Lcd4480WriteRS( CMD );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );
        
        Lcd4480PulseEN( );
        delay( 5 );
        
        Lcd4480WriteRS( CMD );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );
        
        Lcd4480PulseEN( );        
        delay( 1 );        
        
        Lcd4480WriteRS( CMD );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );   
        
        Lcd4480PulseEN( );
        delay( 1 );        
        
        Lcd4480WriteRS( CMD );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 0 );           
        
        Lcd4480PulseEN( );  
        delay( 1 );     

        Lcd4480CmdWrite( LCD_FUNCTION_4BIT | LCD_FUNCTION_2LINE | LCD_FUNCTION_5X7 );
        
        Lcd4480CmdWrite( LCD_DISPLAY_OFF );
        
        Lcd4480CmdWrite( LCD_CLEAR );
        
        Lcd4480CmdWrite( LCD_ENTRY_INCREASE | LCD_ENTRY_NOSHIFT );
                
        bInit = 1;
    }
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Write( const char* pcMsg )
{    
    while( *pcMsg != '\0' )
    {
         Lcd4480DataWrite( *pcMsg++ );    
    }
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Clear( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Home( void )
{
    Lcd4480CmdWrite( LCD_HOME );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480SetCursor( uint8_t x, uint8_t y )
{
    uint8_t data;
 
    switch (y)
    {
        case 1:    // 1. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE1 + x;
            break;
 
        case 2:    // 2. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE2 + x;
            break;
 
        case 3:    // 3. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE3 + x;
            break;
 
        case 4:    // 4. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE4 + x;
            break;
 
        default:
            return LCD_ERROR;                                   // für den Fall einer falschen Zeile
    }
 
    Lcd4480CmdWrite( data );    
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Cursor( void )
{    
    Lcd4480CmdWrite( LCD_CURSOR_ON );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoCursor( void )
{
   Lcd4480CmdWrite( LCD_CURSOR_OFF );
    
   return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Blink( void )
{
    Lcd4480CmdWrite( LCD_BLINKING_ON );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoBlink( void )
{
    Lcd4480CmdWrite( LCD_BLINKING_OFF );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Display( void )
{
    Lcd4480CmdWrite( LCD_DISPLAYON );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoDisplay( void )
{
    Lcd4480CmdWrite( LCD_DISPLAYOFF );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollDisplayLeft( void )
{
    Lcd4480CmdWrite( LCD_SHIFT_LEFT );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollDisplayRight( void )
{
    Lcd4480CmdWrite( LCD_SHIFT_RIGHT );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollAutoScroll( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollNoAutoScroll( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollLeftToRight( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollRightToLeft( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollCreateChar( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------
