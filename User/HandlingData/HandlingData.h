#include "stm32f10x.h"
#include "./sdio/bsp_sdio_sdcard.h"
#include "ff.h"
#include "stdio.h"
#include "./led/bsp_led.h"


uint8_t logdata(void);
void Save2SD(void);
void SDCard_Init(void);

