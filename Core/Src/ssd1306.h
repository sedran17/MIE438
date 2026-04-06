/*
 * ssd1306.h
 *
 *  Created on: Mar 30, 2026
 *      Author: Mark VI
 */

#ifndef SRC_SSD1306_H_
#define SRC_SSD1306_H_

#include <stdint.h>

extern uint8_t buffer[1024];   // declaration here so it can be accessed in main and ssd1306.c

void SSD1306_Init(void);
void SSD1306_Draw(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_Update(void);

#endif /* SRC_SSD1306_H_ */
