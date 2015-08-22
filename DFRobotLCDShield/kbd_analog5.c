#include "util.h"
#include "ttimer.h"
#include "kbd_analog5.h"
#include "inc/hw_types.h"
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom_map.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>


//-----------------------------------------------------------------------------

#define KB_AN_ADC_SYSCTL   (SYSCTL_PERIPH_ADC0)
#define KB_AN_ADC_BASE     (ADC0_BASE)
#define KB_AN_ADC_SEQUENCE (1)
#define KB_AN_ADC_CHANNEL  (ADC_CTL_CH4)

#define KB_AN_GPIO_SYSCTL  (SYSCTL_PERIPH_GPIOD)
#define KB_AN_GPIO_BASE    (GPIO_PORTD_BASE)
#define KB_AN_GPIO_PIN     (GPIO_PIN_7)

#define KB_AN_TIMER        (2*TTIMER_1MS_INTERVAL)

//-----------------------------------------------------------------------------

/* Key values
    Select 	: 3380
    Left	: 2500
    Down	: 1570
    Up		: 620
    Right	: 0
*/

#define KB_AN_RIGHTKEY_UPPER_LIMIT          (100)
#define KB_AN_RIGHTKEY_LOWER_LIMIT          (0)
#define KB_AN_UPKEY_UPPER_LIMIT             (620+100)
#define KB_AN_UPKEY_LOWER_LIMIT             (620-100)
#define KB_AN_DOWNKEY_UPPER_LIMIT           (1570+100)
#define KB_AN_DOWNKEY_LOWER_LIMIT           (1570-100)
#define KB_AN_LEFTKEY_UPPER_LIMIT           (2500+100)
#define KB_AN_LEFTKEY_LOWER_LIMIT           (2500-100)
#define KB_AN_SELKEY_UPPER_LIMIT            (3380+100)
#define KB_AN_SELKEY_LOWER_LIMIT            (3380-100)
#define KB_AN_NONE_UPPER_LIMIT              (4095)   
#define KB_AN_NONE_LOWER_LIMIT              (4095-100)

//-----------------------------------------------------------------------------

struct _KbAnLimits
{
    uint16_t wUpperLimit;
    uint16_t wLowerLimit;
};

const struct _KbAnLimits kbAnLimits[] = 
{
    [KB_AN_RIGHT] = 
    {
        .wUpperLimit = KB_AN_RIGHTKEY_UPPER_LIMIT,
        .wLowerLimit = KB_AN_RIGHTKEY_LOWER_LIMIT,
    },

    [KB_AN_UP] = 
    {
        .wUpperLimit = KB_AN_UPKEY_UPPER_LIMIT,
        .wLowerLimit = KB_AN_UPKEY_LOWER_LIMIT,
    },
    
    [KB_AN_DOWN] = 
    {
        .wUpperLimit = KB_AN_DOWNKEY_UPPER_LIMIT,
        .wLowerLimit = KB_AN_DOWNKEY_LOWER_LIMIT,
    },

    [KB_AN_LEFT] = 
    {
        .wUpperLimit = KB_AN_LEFTKEY_UPPER_LIMIT,
        .wLowerLimit = KB_AN_LEFTKEY_LOWER_LIMIT,
    },

    [KB_AN_SELECT] = 
    {
        .wUpperLimit = KB_AN_SELKEY_UPPER_LIMIT,
        .wLowerLimit = KB_AN_SELKEY_LOWER_LIMIT,
    },

    [KB_AN_NONE] = 
    {
        .wUpperLimit = KB_AN_NONE_UPPER_LIMIT,
        .wLowerLimit = KB_AN_NONE_LOWER_LIMIT,
    },
    
};

static uint8_t  bInit = 0;          //! initialization variable
static uint32_t dwKbAnHandle = 0;   //!   
static uint32_t dwAnalogKbValue = 0; //! 

//-----------------------------------------------------------------------------

//! task to read the analog input
uint32_t ReadKbAnTask( void* pParam );

//-----------------------------------------------------------------------------

KB_AN_STATUS_t KbAnInit( void )
{
    KB_AN_STATUS_t  xRet = KB_AN_ST_OK;
    uint32_t        dwRet = 0;
    
    if ( bInit == 0 )
    {
        
        dwRet = TTimerRegisterCallBack( KB_AN_TIMER     , 
                                        TimerPeriodic   , 
                                        ReadKbAnTask    ,
                                        NULL            ,
                                        &dwKbAnHandle );

        if ( !dwRet )
        {
            SysCtlPeripheralEnable(KB_AN_ADC_SYSCTL);
            SysCtlPeripheralEnable(KB_AN_GPIO_SYSCTL);
            
            SysCtlPeripheralEnable(KB_AN_GPIO_SYSCTL);
            SysCtlPeripheralEnable(KB_AN_ADC_SYSCTL);

            /*Definindo tipo do pino E7 como entrada do ADC*/
            GPIOPinTypeADC(KB_AN_GPIO_BASE, KB_AN_GPIO_PIN);

            /*CONFIGURAÇÃO DO ADC*/
            /*Configura a fonte do trigger e a sequencia de amostra*/
            ADCSequenceConfigure(KB_AN_GPIO_BASE, KB_AN_ADC_SEQUENCE, ADC_TRIGGER_PROCESSOR, 0);
            /*Passdos para a sequencia de amostra
            Base de dados - numero da seq - passo configurado - configuração do passo*/
            ADCSequenceStepConfigure(KB_AN_ADC_BASE, 
                                     KB_AN_ADC_SEQUENCE, 
                                     0,
                                     ADC_CTL_IE | ADC_CTL_END | KB_AN_ADC_CHANNEL );
            /*Habilita amostragem*/
            ADCSequenceEnable(KB_AN_ADC_BASE, KB_AN_ADC_SEQUENCE);    
            
            TTimerStart( dwKbAnHandle );
            
            bInit = 1;            
            
        }
    }
    
    return xRet;
}

//-----------------------------------------------------------------------------

uint8_t KbAnReadKey( KB_AN key )
{
    uint8_t bRetKb=0;
    
    switch( key )
    {
        case KB_AN_RIGHT :
        case KB_AN_UP    :
        case KB_AN_DOWN  :
        case KB_AN_LEFT  :
        case KB_AN_SELECT:
        {
            if ( (dwAnalogKbValue > kbAnLimits[key].wLowerLimit) && (dwAnalogKbValue <= kbAnLimits[key].wUpperLimit))
            {
                bRetKb = 1;
            }
        }
        break;
        
        default:
        {
            bRetKb = 0;
        }
        break;
    }
    
    return bRetKb;
}

//-----------------------------------------------------------------------------

KB_AN_STATUS_t KbAnRegisterCallBack( KB_AN key, callbackkban_func cb_kban )
{
    KB_AN_STATUS_t xRet = KB_AN_ST_ERROR;
    
    return xRet;
}

//-----------------------------------------------------------------------------

uint32_t ReadKbAnTask( void* pParam )
{
    static BYTE bKbAnState = 0;
    static uint32_t dwADCValue = 0;
    
    switch( bKbAnState )
    {
        case 0:
        {
            ADCIntClear(KB_AN_ADC_BASE, KB_AN_ADC_SEQUENCE);
            ADCProcessorTrigger(KB_AN_ADC_BASE, KB_AN_ADC_SEQUENCE);
            
            bKbAnState = 1;
        }
        break;
            
        case 1:
        {
            bKbAnState = 1;
            if ( ADCIntStatus(KB_AN_ADC_BASE, KB_AN_ADC_SEQUENCE, false) )
            {
                bKbAnState = 2;
            }                 
        }
        break;
        
        case 2:
        {
            ADCSequenceDataGet(KB_AN_ADC_BASE, KB_AN_ADC_SEQUENCE, &dwADCValue);
            bKbAnState = 3;
        }
        break;
        
        case 3:
        {
            dwAnalogKbValue = dwADCValue;
            bKbAnState = 0;
        }
        break;
        
        default:
            bKbAnState = 0;
            break;
      
    }
    
    return 0;
}

//-----------------------------------------------------------------------------
