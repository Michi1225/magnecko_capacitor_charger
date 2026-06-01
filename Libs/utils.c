#include "utils.h"


// LED Utility Functions and Color Definitions
#ifdef LED_UTILS
void setStatusLED(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t intensity = 100; // Default intensity (0-255)
    //Clamp intensity to 0-255
    if(intensity > 255) intensity = 255;
    //Scale colors by intensity
    uint8_t r_scaled = (r * intensity) / 255;
    uint8_t g_scaled = (g * intensity) / 255;
    uint8_t b_scaled = (b * intensity) / 255;

    //Assuming Timer period is 255 for 8-bit resolution
    TIM3->CCR1 = g_scaled; //Green
    TIM3->CCR2 = b_scaled; //Blue
    TIM3->CCR4 = r_scaled; //Red
}

void disableStatusLED()
{
    setStatusLED(0, 0, 0);
}

void setStatusLEDWarning()
{
    //Yellow color for warning
    setStatusLED(255, 255, 0);
}

void setStatusLEDError()
{
    //Red color for error
    setStatusLED(255, 0, 0);
}

void setStatusLEDHex(uint32_t hexColor)
{
    uint8_t r = (hexColor >> 16) & 0xFF;
    uint8_t g = (hexColor >> 8) & 0xFF;
    uint8_t b = hexColor & 0xFF;
    setStatusLED(r, g, b);
}

#endif

