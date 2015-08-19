

#ifndef _KBD_ANALOG5_H_
#define _KBD_ANALOG5_H_


#include <stdint.h>

typedef enum 
{
    KB_AN_RIGHT = 1,
    KB_AN_UP    ,
    KB_AN_DOWN  ,
    KB_AN_LEFT  ,
    KB_AN_SELECT,
    KB_AN_NONE  ,
} KB_AN;

typedef enum 
{
    KB_AN_OK    ,
    KB_AN_ERROR ,
} KB_AN_STATUS_t;

typedef void (*callbackkban_func)( void );

/**
 *  @brief  initialize the analog keyboard hardware
 *  @return KB_AN_OK    initialization OK
 *  @return KB_AN_ERROR initialization ERROR
 */
KB_AN_STATUS_t KbAnInit( void );

/**
 *  @brief  read the key value
 *  @return KB_AN_OK    initialization OK
 *  @return KB_AN_ERROR initialization ERROR
 */
KB_AN KbAnRead( KB_AN key );

/**
 *  @brief  read the key value
 *  @return KB_AN_OK    initialization OK
 *  @return KB_AN_ERROR initialization ERROR
 */
KB_AN_STATUS_t KbAnRegisterCallBack( KB_AN key, callbackkban_func cb_kban );


#endif //~_KBD_ANALOG5_H_

