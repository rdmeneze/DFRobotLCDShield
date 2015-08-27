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

void Lcd4480DbWrite( uint8_t bData )
{
    if ( bData & (1<<7))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB7].dwBASE, stGpioLcdCfg[LCD_DB7].dwPin, stGpioLcdCfg[LCD_DB7].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB7].dwBASE, stGpioLcdCfg[LCD_DB7].dwPin, ~stGpioLcdCfg[LCD_DB7].dwPin );
    }
    
    if ( bData & (1<<6))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB6].dwBASE, stGpioLcdCfg[LCD_DB6].dwPin, stGpioLcdCfg[LCD_DB6].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB6].dwBASE, stGpioLcdCfg[LCD_DB6].dwPin, ~stGpioLcdCfg[LCD_DB6].dwPin );
    }
    if ( bData & (1<<5))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB5].dwBASE, stGpioLcdCfg[LCD_DB5].dwPin, stGpioLcdCfg[LCD_DB5].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB5].dwBASE, stGpioLcdCfg[LCD_DB5].dwPin, ~stGpioLcdCfg[LCD_DB5].dwPin );
    }
    if ( bData & (1<<4))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB4].dwBASE, stGpioLcdCfg[LCD_DB4].dwPin, stGpioLcdCfg[LCD_DB4].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB4].dwBASE, stGpioLcdCfg[LCD_DB4].dwPin, ~stGpioLcdCfg[LCD_DB4].dwPin );
    }
    if ( bData & (1<<7))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB7].dwBASE, stGpioLcdCfg[LCD_DB7].dwPin, stGpioLcdCfg[LCD_DB7].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB7].dwBASE, stGpioLcdCfg[LCD_DB7].dwPin, ~stGpioLcdCfg[LCD_DB7].dwPin );
    }
    
    if ( bData & (1<<6))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB6].dwBASE, stGpioLcdCfg[LCD_DB6].dwPin, stGpioLcdCfg[LCD_DB6].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB6].dwBASE, stGpioLcdCfg[LCD_DB6].dwPin, ~stGpioLcdCfg[LCD_DB6].dwPin );
    }
    if ( bData & (1<<5))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB5].dwBASE, stGpioLcdCfg[LCD_DB5].dwPin, stGpioLcdCfg[LCD_DB5].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB5].dwBASE, stGpioLcdCfg[LCD_DB5].dwPin, ~stGpioLcdCfg[LCD_DB5].dwPin );
    }
    if ( bData & (1<<4))
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB4].dwBASE, stGpioLcdCfg[LCD_DB4].dwPin, stGpioLcdCfg[LCD_DB4].dwPin );
    }
    else
    {
        GPIOPinWrite( stGpioLcdCfg[LCD_DB4].dwBASE, stGpioLcdCfg[LCD_DB4].dwPin, ~stGpioLcdCfg[LCD_DB4].dwPin );
    }    
    
    return;
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
