/*
 * ssd1306.c
 *
 *  Created on: Mar 30, 2026
 *      Author: Mark VI
 */


// Note that these functions are set up to draw individual pixels
// you should create a full picture in memory then update the display (or else it will be soooo slow)


#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include "characters.h"

#define ROW_SIZE 256

extern I2C_HandleTypeDef hi2c1;

uint8_t buffer[1024];



void SSD1306_Write(uint8_t cmd){
	uint8_t data[2];
	data[0] = 0x00;
	data[1] = cmd;
	HAL_I2C_Master_Transmit(&hi2c1, 0x3C << 1, data, 2, HAL_MAX_DELAY);
}

void SSD1306_Init(void){
	HAL_Delay(100);

	SSD1306_Write(0xAE); // Display OFF
	SSD1306_Write(0x20); // Memory mode
	SSD1306_Write(0x00); // Horizontal addressing

	SSD1306_Write(0xB0); // Page start
	SSD1306_Write(0xC8); // COM scan direction
	SSD1306_Write(0x00); // Low column
	SSD1306_Write(0x10); // High column

	SSD1306_Write(0x40); // Start line
	SSD1306_Write(0x81); // Contrast
	SSD1306_Write(0x7F);

	SSD1306_Write(0xA1); // Segment remap
	SSD1306_Write(0xA6); // Normal display
	SSD1306_Write(0xA8); // Multiplex
	SSD1306_Write(0x3F);

	SSD1306_Write(0xA4);
	SSD1306_Write(0xD3); // Offset
	SSD1306_Write(0x00);

	SSD1306_Write(0xD5); // Clock
	SSD1306_Write(0x80);

	SSD1306_Write(0xD9); // Pre-charge
    SSD1306_Write(0xF1);

    SSD1306_Write(0xDA); // COM pins
    SSD1306_Write(0x12);

    SSD1306_Write(0xDB); // VCOM detect
    SSD1306_Write(0x40);

    SSD1306_Write(0x8D); // Charge pump
    SSD1306_Write(0x14);

    SSD1306_Write(0xAF); // Display ON
}


void SSD1306_Update(void) {
    for (uint8_t page = 0; page < 8; page++) {
        SSD1306_Write(0xB0 + page); // Set page
        SSD1306_Write(0x00);        // Low col
        SSD1306_Write(0x10);        // High col

        uint8_t data[129];
        data[0] = 0x40; // Data mode

        for (uint8_t i = 0; i < 128; i++) {
            data[i + 1] = buffer[page * 128 + i]; // append data with the entire buffer
        }
        /*
        * NOTE: this function is pretty resource/time intensive and updates the *entire screen at once*
        * because of this it is best practice to make all required changes then call an update instead of updating every step
        * this is why no higher-level functions call this automatically
        */

        HAL_I2C_Master_Transmit(&hi2c1, 0x3C << 1, data, 129, HAL_MAX_DELAY);
    }
}

// Due to how we print/store characters, editing an individual pixel was never actually used
/*
void SSD1306_Draw(uint8_t x, uint8_t y, uint8_t colour) {
    if (x >= 128 || y >= 64) return;  // Need to exclude pixels off the edges of the display (this will crop anything that is too big)

    if (colour)
        buffer[x + (y / 8) * 128] |= (1 << (y % 8));
    else
        buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}
*/


void SSD1306_Fill(uint8_t colour){
	uint8_t fill = colour * 0xFF; //used to save doing this every iteration
	for(int i = 0; i<1024;i++){
		buffer[i] = fill;
	}
}

// prints a specific 16x1 area of the screen
// note that this means there are *only 4 rows* under this interpretation (thus why we multiply by 256 instead of the true screen width (256)
// this was done to line up with how we designed the screens (see screen.h)
void SSD1306_Print(const uint8_t *top, const uint8_t *bottom, int col, int row){
	for(int i = 0; i < 8; i++){
		buffer[col + i + (row * ROW_SIZE)] = top[i];
		buffer[col + i + 128 + (row * ROW_SIZE)] = bottom[i];
	}
}

// Used so you can enter a whole word at a time
void SSD1306_Word(const int *characters, size_t len, int row){
	int col = 2;
	for(size_t i = 0; i < len; i++){
		SSD1306_Print(top[characters[i]], bottom[characters[i]], col, row);
		col += 10; // move the column over 10 (8 for the character plus two for letter spacing)
	}
}

// used to highlight a row when it is selected
void SSD1306_Invert(int row){
	for(int i = 0; i < ROW_SIZE; i++){
		buffer[i + (row*ROW_SIZE)] = ~buffer[i + (row*ROW_SIZE)]; // inverts the pixel
	}
}

