#ifndef INC_GAMEDATA_H_
#define INC_GAMEDATA_H_

#include <GameObject.h>

//#define SCALE 1.

typedef enum
{
	Menu,
	Game,
	Crash,
} gameState_t;

gameObject_t objects[] = {
		{ LCD_COLOR_CYAN, 100., 200., 20, 30 },
		{ LCD_COLOR_RED, 60., 30., 20, 30 },
		{ LCD_COLOR_GREEN, 120., 60., 20, 30 },
		{ LCD_COLOR_BLUE, 180., 90., 20, 30 },
};
gameObject_t * player = &objects[0];
gameObject_t * obsticles[3] = {
		&objects[1], &objects[2], &objects[3]
};

uint32_t address, levelLength;
int8_t inputX, inputY, printLine;
gameState_t gameState = Game;

#endif
