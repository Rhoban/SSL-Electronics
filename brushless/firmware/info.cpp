#include "info.h"
#include "hardware.h"


bool flashErasePage(u32 pageAddr) {
  u32 rwmVal = GET_REG(FLASH_CR);
  rwmVal = FLASH_CR_PER;
  SET_REG(FLASH_CR, rwmVal);

  while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}
  SET_REG(FLASH_AR, pageAddr);
  SET_REG(FLASH_CR, FLASH_CR_START | FLASH_CR_PER);
  while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}

  /* todo: verify the page was erased */

  rwmVal = 0x00;
  SET_REG(FLASH_CR, rwmVal);

  return true;
}

bool flashErasePages(u32 pageAddr, u16 n) {
  while (n-- > 0) {
    if (!flashErasePage(pageAddr + 0x400 * n)) {
      return false;
    }
  }

  return true;
}

bool flashWriteWord(u32 addr, u32 word) {
  vu16 *flashAddr = (vu16 *)addr;
  vu32 lhWord = (vu32)word & 0x0000FFFF;
  vu32 hhWord = ((vu32)word & 0xFFFF0000) >> 16;

  u32 rwmVal = GET_REG(FLASH_CR);
  SET_REG(FLASH_CR, FLASH_CR_PG);

  /* apparently we need not write to FLASH_AR and can
     simply do a native write of a half word */
  while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {
    // digitalWrite(LED_PIN, LOW);
    // delay_us(1000);
    // digitalWrite(LED_PIN, HIGH);
    // delay_us(10000);
  }
  *(flashAddr + 0x01) = (vu16)hhWord;
  while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {
    // digitalWrite(LED_PIN, LOW);
    // delay_us(1000);
    // digitalWrite(LED_PIN, HIGH);
    // delay_us(10000);
  }
  *(flashAddr) = (vu16)lhWord;
  while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {
    // digitalWrite(LED_PIN, LOW);
    // delay_us(1000);
    // digitalWrite(LED_PIN, HIGH);
    // delay_us(10000);
  }

  rwmVal &= 0xFFFFFFFE;
  SET_REG(FLASH_CR, rwmVal);

  /* verify the write */
  if (*(vu32 *)addr != word) {
    return false;
  }

  return true;
}

void flashLock() {
  /* take down the HSI oscillator? it may be in use elsewhere */

  /* ensure all FPEC functions disabled and lock the FPEC */
  SET_REG(FLASH_CR, 0x00000080);
}

void flashUnlock() {
  /* unlock the flash */
  SET_REG(FLASH_KEYR, FLASH_KEY1);
  SET_REG(FLASH_KEYR, FLASH_KEY2);
}

/* Minimum_Source*/

void setupFLASH() {
  /* configure the HSI oscillator */
  if ((pRCC->CR & 0x01) == 0x00) {
    u32 rwmVal = pRCC->CR;
    rwmVal |= 0x01;
    pRCC->CR = rwmVal;
  }

  /* wait for it to come on */
  while ((pRCC->CR & 0x02) == 0x00) {}
}

bool writeChunk(u32 *ptr, int size, const char *data)
{
  flashErasePage((u32)(ptr));

  for (int i = 0; i<size; i = i + 4) {
    if (!flashWriteWord((u32)(ptr++), *((u32 *)(data + i)))) {
      return false;
    }
  }

  return true;
}

void info_set(motor_info data)
{
  noInterrupts();
  setupFLASH();
  flashUnlock();

  int n = sizeof(data);

  for (int i=0; i<n; i+=PAGE_SIZE) {
    int size = 0;
    u32* chunk = (u32 *)(INFO_FLASH_ADDR + i);

    size = n-i;
    if (size > PAGE_SIZE) size = PAGE_SIZE;

    if (!writeChunk(chunk, size, &((char*)&data)[i])) {
      break;
    }
  }

  flashLock();
  interrupts();
}
