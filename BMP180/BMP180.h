#ifndef _BMP180_H_
#define _BMP180_H_

#include "defs.h"

typedef struct SFE_BMP180 SFE_BMP180;

SFE_BMP180* Bmp180Create( void );

uint32_t Bmp180Init( SFE_BMP180* pBmp180 );

#endif
