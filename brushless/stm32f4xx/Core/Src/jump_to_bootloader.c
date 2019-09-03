/*
g
 * Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
 *
 * This code is inspired by the tutorial :
 * stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32
 * written on April 3, 2017 by TILZOR.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "stm32f4xx_hal.h"
#include <terminal.h>

/**
 * Jumps to the system memory boot from user application.
 */
void jump_to_bootloader(void) {	
	void(*jump_to_the_bootloader)(void);
	
  // The address of the system memory.
  //       
  // For STM32F4XX this adress is 0x1FFF0000.
  // Check the manual reference AN2606 of STM32 to obtain the system 
  // memory of other micro-processor.
  volatile uint32_t system_memory_address = 0x1FFF0000;

	// The address of the system jump is located at 4 bytes + the system memory
  // addess;
	jump_to_the_bootloader = (void (*)(void)) (*((uint32_t *)(system_memory_address + 4)));

  // Reinit RCC, clock, ...
  HAL_RCC_DeInit();
	
	// Reset systick timer.
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

  // __disable_irq(); // This is not needed.
	
  // Remap system memory to address 0x00000000 in address space.
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	
  // Set main stack pointer.
  // This step must be done last otherwise local variables in this function
  // don't have proper value since stack pointer is located on different position
  // Set direct address location which specifies stack pointer in SRAM location
	__set_MSP(*(uint32_t *)system_memory_address);
	
  // Execute the bootloader
	jump_to_the_bootloader();
}

