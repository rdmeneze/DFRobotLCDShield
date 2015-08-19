#include "kbd_analog5.h"
#include "ttimer.h"
#include "util.h"
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>

//-----------------------------------------------------------------------------

KB_AN_STATUS_t KbAnInit( void )
{
    KB_AN_STATUS_t xRet = KB_AN_ERROR;
    
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
    KB_AN_STATUS_t xRet = KB_AN_ERROR;
    
    return xRet;
}

//-----------------------------------------------------------------------------
