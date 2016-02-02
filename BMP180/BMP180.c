
#include "BMP180.h"

#define N_BMP189 1

/**
 * @brief definition of SFE_BMP180 struct
 */
struct SFE_BMP180
{
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    int16_t VB1;
    int16_t VB2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6; 
    double c5;
    double c6;
    double mc;
    double md;
    double x0;
    double x1;
    double x2;
    double y0;
    double y1;
    double y2;
    double p0;
    double p1;
    double p2;
    char _error;
};

static SFE_BMP180 xBmp180[N_BMP189];
static uint8_t bBmp180Count = 0;

/******************************************************************************/

SFE_BMP180* Bmp180Create( void )
{
    SFE_BMP180* pBmp180 = NULL;
    
    pBmp180 = &xBmp180[bBmp180Count];
        
    bBmp180Count++;
    if ( bBmp180Count ==  N_BMP189 )
    {
        bBmp180Count = 0;
    }
    
    return pBmp180;
}

/******************************************************************************/

uint32_t Bmp180Init( SFE_BMP180* pBmp180 )
{
    return 0;
}
