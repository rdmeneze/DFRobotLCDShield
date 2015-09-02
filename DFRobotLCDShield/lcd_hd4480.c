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


//-----------------------------------------------------------------------------

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

void Lcd4480DbWrite( uint8_t bData )
{
    uint8_t i = 7;
    
    Lcd4480WriteRS( DATA );
    
    Lcd4480WriteDB7( bData & (1<<(i--)));
    Lcd4480WriteDB6( bData & (1<<(i--)));
    Lcd4480WriteDB5( bData & (1<<(i--)));
    Lcd4480WriteDB4( bData & (1<<(i--)));
    
    Lcd4480PulseEN( );
    
    Lcd4480WriteDB7( bData & (1<<(i--)));
    Lcd4480WriteDB6( bData & (1<<(i--)));
    Lcd4480WriteDB5( bData & (1<<(i--)));
    Lcd4480WriteDB4( bData & (1<<(i--)));

    Lcd4480PulseEN( );
    
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
    
    Lcd4480WriteDB7( cmd & (1<<(i--)));
    Lcd4480WriteDB6( cmd & (1<<(i--)));
    Lcd4480WriteDB5( cmd & (1<<(i--)));
    Lcd4480WriteDB4( cmd & (1<<(i--)));

    Lcd4480PulseEN( );    
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
            GPIOPinTypeGPIOInput( stGpioLcdCfg[bCounter].dwBASE, stGpioLcdCfg[bCounter].dwPin );
        }
        
        delay( 30 );
        
        Lcd4480WriteRS( 0 );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );
        
        Lcd4480PulseEN( );
        delay( 5 );
        
        Lcd4480WriteRS( 0 );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );
        Lcd4480PulseEN( );        
        delay( 1 );        
        
        Lcd4480WriteRS( 0 );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 1 );   
        Lcd4480PulseEN( );
        delay( 1 );        
        
        Lcd4480WriteRS( 0 );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 0 );           
        Lcd4480PulseEN( );        
        
        Lcd4480WriteRS( 0 );
        Lcd4480WriteDB7( 0 );
        Lcd4480WriteDB6( 0 );
        Lcd4480WriteDB5( 1 );
        Lcd4480WriteDB4( 0 );           
        Lcd4480PulseEN( );
        
        bInit = 1;
    }
    
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Write( const char* pcMsg )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
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
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480SetCursor( uint8_t bLin, uint8_t bCol )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Cursor( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoCursor( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Blink( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoBlink( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480Display( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480NoDisplay( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollDisplayLeft( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
}

//-----------------------------------------------------------------------------

LCD_STATUS Lcd4480ScrollDisplayRight( void )
{
    LCD_STATUS xRet = LCD_OK;
    
    return xRet;
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
