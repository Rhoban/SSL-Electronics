
#if !defined(INFO_H)
#define INFO_H

#include <stdlib.h>
#include <wirish/wirish.h>


typedef unsigned short       u16;
typedef volatile unsigned short       vu16;
typedef unsigned long       u32;
typedef volatile unsigned long       vu32;
typedef unsigned long       ui32;

#define BOOTLOADER_FLASH   ((u32)0x08000000)
#define PAGE_SIZE          1024

#define SET_REG(addr,val) do { *(vu32*)(addr)=val; } while(0)
#define GET_REG(addr)     (*(vu32*)(addr))


#define RCC   ((u32)0x40021000)
#define FLASH ((u32)0x40022000)

#define RCC_CR      RCC
#define RCC_CFGR    (RCC + 0x04)
#define RCC_CIR     (RCC + 0x08)
#define RCC_AHBENR  (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x1C)

#define FLASH_ACR     (FLASH + 0x00)
#define FLASH_KEYR    (FLASH + 0x04)
#define FLASH_OPTKEYR (FLASH + 0x08)
#define FLASH_SR      (FLASH + 0x0C)
#define FLASH_CR      (FLASH + 0x10)
#define FLASH_AR      (FLASH + 0x14)
#define FLASH_OBR     (FLASH + 0x1C)
#define FLASH_WRPR    (FLASH + 0x20)

#define FLASH_KEY1     0x45670123
#define FLASH_KEY2     0xCDEF89AB
#define FLASH_RDPRT    0x00A5
#define FLASH_SR_BSY   0x01
#define FLASH_CR_PER   0x02
#define FLASH_CR_PG    0x01
#define FLASH_CR_START 0x40

typedef struct {
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC)


struct motor_info
{
  int tare_value;
};


bool flashErasePage(u32 pageAddr);
bool flashErasePages(u32 pageAddr, u16 n);
bool flashWriteWord(u32 addr, u32 word);
void flashLock();
void flashUnlock();
void setupFLASH();
bool writeChunk(u32 *ptr, int size, const char *data);

void info_set(motor_info data);


#endif
