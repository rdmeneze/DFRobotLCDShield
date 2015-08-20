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

#define KB_AN_TIMER        (50*TTIMER_1MS_INTERVAL)

//-----------------------------------------------------------------------------

static uint8_t  bInit = 0;          //! initialization variable
static uint32_t dwKbAnHandle = 0;   //!   
static uint16_t wAnalogKbValue = 0; //! 

//-----------------------------------------------------------------------------

uint32_t ReadKbAnTask( void* pParam );

//-----------------------------------------------------------------------------

KB_AN_STATUS_t KbAnInit( void )
{
    KB_AN_STATUS_t  xRet = 0;
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

KB_AN KbAnRead( KB_AN key )
{
    KB_AN xRetKb = KB_AN_NONE;
    
    return xRetKb;
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
    }
    
    return 0;
}

//-----------------------------------------------------------------------------
