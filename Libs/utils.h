#pragma once
#include "main.h"
#include "tim.h"

#define LED_UTILS


// LED Utility Functions and Color Definitions
#ifdef LED_UTILS
    #define COLOUR_RED     0xFF0000
    #define COLOUR_GREEN   0x00FF00
    #define COLOUR_BLUE    0x0000FF
    #define COLOUR_YELLOW  0xFFFF00
    #define COLOUR_CYAN    0x00FFFF
    #define COLOUR_MAGENTA 0xFF00FF
    #define COLOUR_WHITE   0xFFFFFF


    void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
    void disableStatusLED();
    void setStatusLEDWarning();
    void setStatusLEDError();
    void setStatusLEDHex(uint32_t hexColor);
#endif



