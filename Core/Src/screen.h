/*
 * screen.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Mark VI
 */

#ifndef SRC_SCREEN_H_
#define SRC_SCREEN_H_

// Screen data type
/*
 * As we designed each character as 16 pixels tall we chose to divide the screen into 4 lines of text
 * Additionally we wanted text highlighting to make your selection obvious
 * Therefore we track the screens internal state separately from the overall display state
 * To reduce the amount of screen change logic required the screen are essentially stored as a circular linked list
 * this allows transitions to be logic-free/simplifies the requirement to track the state
 */
struct Screen{
	int* line1; // first line of text
	int len1; // length of above
	int* line2;
	int len2;
	int* line3;
	int len3;
	int* line4;
	int len4;
	int state; // internal screen state (line highlighted)
	int size; // amount of *USER SELECTABLE* lines on the screen (not total lines with text)
	int invert_offset; // used if we never want the first line/lines to be highlighted
	struct Screen *next; // screen after this one
	struct Screen *previous; // screen before
};

#endif /* SRC_SCREEN_H_ */
