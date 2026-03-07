#include "main.h"

#define SWO_SPEED 5E6 // 4Mhz
void SWD_Init(void)
{
  *(__IO uint32_t*)(0x5C001004) |= 0x00700000; // DBGMCU_CR D3DBGCKEN D1DBGCKEN TRACECLKEN
 
  //UNLOCK FUNNEL
  *(__IO uint32_t*)(0x5C004FB0) = 0xC5ACCE55; // SWTF_LAR
  *(__IO uint32_t*)(0x5C003FB0) = 0xC5ACCE55; // SWO_LAR
 
  //SWO current output divisor register
  //This divisor value (0x000000C7) corresponds to 400Mhz
  //To change it, you can use the following rule
  // value = (CPU Freq/sw speed )-1
  // devided by 2 because swo runs on half the System clock
   *(__IO uint32_t*)(0x5C003010) = ((SystemCoreClock / SWO_SPEED / 2) - 1); // SWO_CODR
 
  //SWO selected pin protocol register
   *(__IO uint32_t*)(0x5C0030F0) = 0x00000002; // SWO_SPPR
 
  //Enable ITM input of SWO trace funnel
   *(__IO uint32_t*)(0x5C004000) |= 0x00000007; // SWFT_CTRL (PORT 0, 1 and 2)
 
  //RCC_AHB4ENR enable GPIOB clock
   *(__IO uint32_t*)(0x580244E0) |= 0x00000002;
 
  // Configure GPIOB pin 3 as AF
   *(__IO uint32_t*)(0x58020400) = (*(__IO uint32_t*)(0x58020400) & 0xffffff3f) | 0x00000080;
 
  // Configure GPIOB pin 3 Speed
   *(__IO uint32_t*)(0x58020408) |= 0x00000080;
 
  // Force AF0 for GPIOB pin 3
   *(__IO uint32_t*)(0x58020420) &= 0xFFFF0FFF;
}