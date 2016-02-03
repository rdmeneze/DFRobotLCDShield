/**
 * @file    lcd_hd44780.h
 * @brief   
 * @author  Rafael Dias <rdmeneze@gmail.com>
 * @date    ago/2015
 */

#include "lcd_hd44780.h"
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

//-----------------------------------------------------------------------------

// LCD module information 
#define LCD_DDADR_LINE1 0x00        // start of line 1 (16x2)/(20x2)  
#define LCD_DDADR_LINE2 0x40        // start of line 2 (16x2)/(20x2)   
//#define LCD_LINETHREE 0x14        // start of line 3 (20x4) 
//#define LCD_LINEFOUR 0x54         // start of line 4 (20x4) 
#define LCD_DDADR_LINE3 0x10        // start of line 3 (16x4) 
#define LCD_DDADR_LINE4 0x50        // start of line 4 (16x4)

typedef struct 
{
    uint8_t bLin; 
    uint8_t bCol;
    uint8_t bCurLin;
    uint8_t bCurCol;
} TLcdInfo;

// LCD instructions 

typedef enum
{
    LCD_CLEAR_CMD 			    = (1 << 0), 	//	0b00000001	// replace all characters with ASCII 'space' 
    LCD_HOME_CMD 			    = (1 << 1), 	// 	0b00000010	// return cursor to first position on first line 
    
    // Set Display ---------------- 0b00001xxx  
    LCD_DISPLAY_CMD             = (1 << 3),
        LCD_DISPLAY_BLINK_ON    = LCD_DISPLAY_CMD | (1 << 0), 
        LCD_DISPLAY_BLINK_OFF   = LCD_DISPLAY_CMD           , 
        LCD_DISPLAY_CURSOR_ON   = LCD_DISPLAY_CMD | (1 << 1),
        LCD_DISPLAY_CURSOR_OFF  = LCD_DISPLAY_CMD           ,
        LCD_DISPLAY_ON          = LCD_DISPLAY_CMD | (1 << 2),  
        LCD_DISPLAY_OFF         = LCD_DISPLAY_CMD           ,  
    
    // Set Entry Mode ------------- 0b000001xx
    LCD_ENTRY_CMD           = (1 << 2)                  ,
        LCD_ENTRY_DECREASE  = LCD_ENTRY_CMD             ,
        LCD_ENTRY_INCREASE  = LCD_ENTRY_CMD | (1 << 1)  ,
        LCD_ENTRY_NOSHIFT   = LCD_ENTRY_CMD             ,
        LCD_ENTRY_SHIFT     = LCD_ENTRY_CMD | (1 << 0)  ,
   
    // Set Shift ------------------ 0b0001xxxx
    LCD_SHIFT_CMD           = (1 << 4)                  , 
        LCD_CURSOR_SHIFT    = LCD_SHIFT_CMD             ,
        LCD_DISPLAY_SHIFT   = LCD_SHIFT_CMD | (1 << 3)  ,
        LCD_SHIFT_LEFT      = LCD_SHIFT_CMD             ,
        LCD_SHIFT_RIGHT     = LCD_SHIFT_CMD | (1 << 2)  ,
    
    // Set Function --------------- 0b001xxxxx
    LCD_FUNCTION_CMD        = (1 << 5), 
        LCD_FUNCTION_4BIT   = LCD_FUNCTION_CMD | 0x00,
        LCD_FUNCTION_8BIT   = LCD_FUNCTION_CMD | 0x10, 
        LCD_FUNCTION_1LINE  = LCD_FUNCTION_CMD | 0x00, 
        LCD_FUNCTION_2LINE  = LCD_FUNCTION_CMD | 0x08, 
        LCD_FUNCTION_5X7    = LCD_FUNCTION_CMD | 0x00,
        LCD_FUNCTION_5X10   = LCD_FUNCTION_CMD | 0x04,
        
    LCD_SOFT_RESET          = 0x30,
    
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
static TLcdInfo lcd = {0,0};

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

void Lcd44780WriteDB7( uint8_t bData )
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

void Lcd44780WriteDB6( uint8_t bData )
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

void Lcd44780WriteDB5( uint8_t bData )
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

void Lcd44780WriteDB4( uint8_t bData )
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

void Lcd44780WriteEN( uint8_t bData )
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

#define Lcd44780PulseEN()    \
    do                      \
    {                       \
        Lcd44780WriteEN(1);  \
        delay(2);           \
        Lcd44780WriteEN(0);  \
        delay(2);           \
    } while( 0 )
    
//-----------------------------------------------------------------------------

void Lcd44780WriteRS( RS_TYPE rs )
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

void Lcd44780WriteCmd4bit( uint8_t cmd )
{
    Lcd44780WriteRS( CMD );
  
    Lcd44780WriteDB7( cmd & (1<<3));
    Lcd44780WriteDB6( cmd & (1<<2));
    Lcd44780WriteDB5( cmd & (1<<1));
    Lcd44780WriteDB4( cmd & (1<<0));

    Lcd44780PulseEN( );  
    return;
}

//-----------------------------------------------------------------------------

void Lcd44780WriteData4bit( uint8_t data )
{
    Lcd44780WriteRS( DATA );
  
    Lcd44780WriteDB7( data & (1<<3));
    Lcd44780WriteDB6( data & (1<<2));
    Lcd44780WriteDB5( data & (1<<1));
    Lcd44780WriteDB4( data & (1<<0));

    Lcd44780PulseEN( );  
    return;
}

//-----------------------------------------------------------------------------

void Lcd44780DataWrite( uint8_t bData )
{   
    Lcd44780WriteData4bit( bData >> 4 );
    Lcd44780WriteData4bit( bData & 0x0F );
    
    return;
}

//-----------------------------------------------------------------------------

void Lcd44780WriteCmd( uint8_t cmd )
{    
    Lcd44780WriteCmd4bit( cmd >> 4 );    
    Lcd44780WriteCmd4bit( cmd & 0x0F );    
    
    return;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Init( uint8_t bLin, uint8_t bCol )
{
    LCD_STATUS xRet = LCD_ERROR;
    
    if ( !bInit )
    {
		uint8_t bCounter;
        
		// configura os GPIOs 
        for ( bCounter = 0; bCounter < GET_ARRAY_LEN( stGpioLcdCfg ); bCounter++ )
        {
            SysCtlPeripheralEnable( stGpioLcdCfg[bCounter].dwSYSCTL );
            GPIOPinTypeGPIOOutput( stGpioLcdCfg[bCounter].dwBASE, stGpioLcdCfg[bCounter].dwPin );
            GPIOPinWrite( stGpioLcdCfg[bCounter].dwBASE,stGpioLcdCfg[bCounter].dwPin, ~stGpioLcdCfg[bCounter].dwPin ); 
        }
        
        
        if ( (bLin == 2) || (bLin == 4) )
        {
            lcd.bLin = bLin;
            lcd.bCol = bCol;
           
            delay( 100 );
            
            /* Try to set 4bit mode */
            Lcd44780WriteCmd4bit(0x03);
            delay( 5 );
            
            /* Second try */
            Lcd44780WriteCmd4bit(0x03);
            delay( 5 );

            /* Third goo! */
            Lcd44780WriteCmd4bit(0x03);
            delay( 5 );
            
            /* Set 4-bit interface */
            Lcd44780WriteCmd4bit(0x02);
            delay(1);
            
            Lcd44780WriteCmd( LCD_FUNCTION_4BIT | LCD_FUNCTION_2LINE | LCD_FUNCTION_5X7 );
            
            Lcd44780WriteCmd( LCD_DISPLAY_ON );
            
            Lcd44780Clear( );
            
            Lcd44780WriteCmd( LCD_ENTRY_INCREASE | LCD_ENTRY_NOSHIFT );
                            
            bInit = 1;
        }
    }
    
    return xRet;
}

//-----------------------------------------------------------------------------

uint8_t Lcd44780GetLin(void)
{
    return lcd.bLin;
}

//-----------------------------------------------------------------------------

uint8_t Lcd44780GetCol(void)
{
    return lcd.bCol;
}

//-----------------------------------------------------------------------------

uint8_t Lcd44780GetCurLin(void)
{
    return lcd.bCurLin;
}

//-----------------------------------------------------------------------------

uint8_t Lcd44780GetCurCol(void)
{
    return lcd.bCurCol;
}


//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Write( const char* pcMsg )
{    
    while( *pcMsg != '\0' )
    {
        switch( *pcMsg )
        {
            case '\r': 
            case '\n':  
            {                
            
                lcd.bCurLin++;
                if ( lcd.bCurLin == lcd.bLin )
                    lcd.bCurLin = 0;
                
                lcd.bCurCol = 0;
                
                Lcd44780SetCursor( lcd.bCurLin, lcd.bCurCol );
                pcMsg++;
                break;
            }

            default:
            {
                lcd.bCurCol++;
                if ( lcd.bCurCol == (lcd.bCol+1) )
                {
                    lcd.bCurCol = 0;
                    lcd.bCurLin++;
                    
                    if ( lcd.bCurLin == lcd.bLin )
                        lcd.bCurLin = 0;
                    
                    Lcd44780SetCursor( lcd.bCurLin, lcd.bCurCol );
                    
                }
                
                Lcd44780DataWrite( *pcMsg++ );    
                break;
            }
        }
    }
    
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Clear( void )
{
    Lcd44780WriteCmd( LCD_CLEAR_CMD );
    Lcd44780SetCursor( 0, 0 );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Home( void )
{
    Lcd44780WriteCmd( LCD_HOME_CMD );
    
    lcd.bCurCol= 0;
    lcd.bCurLin = 0;
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780SetCursor( uint8_t x, uint8_t y )
{
    uint8_t data;
    LCD_STATUS xRet = LCD_OK;
 
    switch (x)
    {
        case 0:    // 1. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE1 + y;
            break;
 
        case 1:    // 2. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE2 + y;
            break;
 
        case 2:    // 3. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE3 + y;
            break;
 
        case 3:    // 4. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE4 + y;
            break;
 
        default:
            xRet = LCD_ERROR;                                   // für den Fall einer falschen Zeile
    }
 
    if ( xRet == LCD_OK )
    {
        lcd.bCurLin = x;
        lcd.bCurCol = y;
        Lcd44780WriteCmd( data );    
    }
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Cursor( void )
{    
    Lcd44780WriteCmd( LCD_DISPLAY_CURSOR_ON );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780NoCursor( void )
{
   Lcd44780WriteCmd( LCD_DISPLAY_CURSOR_OFF );
    
   return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Blink( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_BLINK_ON );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780NoBlink( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_BLINK_OFF );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780Display( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_ON );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780NoDisplay( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_OFF );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollDisplayLeft( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_SHIFT | LCD_SHIFT_LEFT );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollDisplayRight( void )
{
    Lcd44780WriteCmd( LCD_DISPLAY_SHIFT | LCD_SHIFT_RIGHT );

    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollAutoScroll( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollNoAutoScroll( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollLeftToRight( void )
{
    Lcd44780WriteCmd( LCD_CURSOR_SHIFT | LCD_SHIFT_RIGHT );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollRightToLeft( void )
{
    Lcd44780WriteCmd( LCD_CURSOR_SHIFT | LCD_SHIFT_LEFT );
    
    return LCD_OK;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780ScrollCreateChar( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780BackLightOn( void )
{
    LCD_STATUS xRet = LCD_ERROR;
    
    if ( bInit )
    {
        const PIN_LCD pin = LCD_BK;
        
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, stGpioLcdCfg[pin].dwPin );
        
    }    
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd44780BackLightOff( void )
{
    LCD_STATUS xRet = LCD_ERROR;

    if ( bInit )
    {
        const PIN_LCD pin = LCD_BK;
        
        GPIOPinWrite( stGpioLcdCfg[pin].dwBASE,stGpioLcdCfg[pin].dwPin, ~stGpioLcdCfg[pin].dwPin ); 
    }    

    return xRet;
}

//-----------------------------------------------------------------------------
