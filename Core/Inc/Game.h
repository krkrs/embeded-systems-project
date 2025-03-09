#ifndef INC_GAME_H_
#define INC_GAME_H_

#include <inttypes.h>
#include "stm32f429i_discovery_lcd.h"

extern volatile int SwitchLayerCmd;

void GameInit();
void GameLoop();
void Inputs();
void SwitchLayersSync(void);
void SwitchLayers(void);

void DrawScene(void);

#endif /* INC_GAME_H_ */
