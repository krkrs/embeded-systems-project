
#define PLAYER_SPEED 120
#define GAME_SPEED 40

#include <Game.h>
#include <GameData.h>
#include <ScreenData.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"

//extern DAC_HandleTypeDef hdac;
extern LTDC_HandleTypeDef LtdcHandler;

extern TIM_HandleTypeDef htim6;

volatile int SwitchLayerCmd;

void printChar(const char * text) {
	BSP_LCD_DisplayStringAtLine(printLine, (uint8_t*)text);
	printLine++;
}

void printInt(int val) {
	char buf[20];
	sprintf(buf, "%d", val);
	printChar(buf);
}

void printFloat(float val) {
	char buf[20];
	sprintf(buf, "%g", val);
	printChar(buf);
}

void Game_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc)
{
	if (SwitchLayerCmd)
	{
		SwitchLayers();
		SwitchLayerCmd = 0;
	}
}

/* Problem z &hdac
void playNote() {
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1023);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2047);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3071);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
}

void stopNote() {
   HAL_TIM_Base_Stop(&htim6);
}

void CrashScreen() {
  // wynik, pik pik
  playNote();
  HAL_Delay(200);
  stopNote();
  HAL_Delay(200);
  playNote();
  HAL_Delay(200);
  stopNote();
  HAL_Delay(200);
  // czekaj na przycisk
  // restart
}*/

void DisplayInit(void)
{
	BSP_LCD_Init();
	extern LTDC_HandleTypeDef hltdc;
	hltdc.State = HAL_LTDC_STATE_READY;
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,
	LCD_FRAME_BUFFER + BUFFER_OFFSET);

	HAL_LTDC_RegisterCallback(&hltdc, HAL_LTDC_LINE_EVENT_CB_ID, Game_LTDC_LineEventCallback);

	SwitchLayersSync();
	BSP_LCD_Clear(LCD_COLOR_DARKGREEN);
	SwitchLayersSync();
	BSP_LCD_Clear(LCD_COLOR_DARKGREEN);
}

void Inputs() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		inputX = 1;
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
		inputX = -1;
	else
		inputX = 0;
}

void Physics() {
	#define DELTA_T 1.0e-3f
	// obsługa poruszania gracza
	player->x += inputX * PLAYER_SPEED * DELTA_T;
	if (player->x < 0) player->x = 0;
	if (player->x > SCREEN_X - player->width) player->x = SCREEN_X - player->width;
	// obsługa poruszania przeszkód
	for (int i = 0; i < 3; i++) {
		obsticles[i]->y += GAME_SPEED * DELTA_T;
		if (obsticles[i]->y > SCREEN_Y) obsticles[i]->y = -obsticles[i]->width;
	}
}

void Collision() {
	for (int i = 0; i < 3; i++) {
	    if (player->x < obsticles[i]->x + obsticles[i]->width &&
	        player->x + player->width > obsticles[i]->x &&
	        player->y < obsticles[i]->y + obsticles[i]->height &&
	        player->y + player->height > obsticles[i]->y) {
	        gameState = Crash;
	    }
	}
}

void Game_Tim6Callback(TIM_HandleTypeDef *htim)
{
	// zebranie wejść
	Inputs();
	if (gameState == Crash) return;
	// animacja obiektów
	Physics();
	// sprawdzenie kolizji
	Collision();
}

void GameInit()
{
	DisplayInit();
	printChar("Super gra!");
	SwitchLayers();
	HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, Game_Tim6Callback);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_Delay(2000);
	//playNote();
	DrawScene();

}

void SwitchLayersSync(void)
{
	SwitchLayerCmd = 1;
	HAL_LTDC_ProgramLineEvent(&LtdcHandler, 324);
	while (SwitchLayerCmd == 1) {};
}

void SwitchLayers(void)
{
	static uint32_t displayedLayer = LCD_FOREGROUND_LAYER;
	static uint32_t modifiedLayer = LCD_BACKGROUND_LAYER;
	if (displayedLayer == LCD_FOREGROUND_LAYER)
	{
		modifiedLayer = displayedLayer;
		displayedLayer = LCD_BACKGROUND_LAYER;
	} else
	{
		modifiedLayer = displayedLayer;
		displayedLayer = LCD_FOREGROUND_LAYER;
	}
	BSP_LCD_SelectLayer(modifiedLayer);
	address = LtdcHandler.LayerCfg[modifiedLayer].FBStartAdress;

	BSP_LCD_SetLayerVisible(displayedLayer, ENABLE);
	BSP_LCD_SetLayerVisible(modifiedLayer, DISABLE);
}

void GameRestart() {
	player->x = 100.;
	player->y = 200.;
	for (int i = 0; i < 3; i++ )
		obsticles[i]->y = 30*(i+1);
	gameState = Game;
}

void GameTakeOver() {
	HAL_Delay(2000);
	while(inputX != 1) { Inputs(); }
	GameRestart();
}

void DrawScene(void)
{
	BSP_LCD_Clear(LCD_COLOR_DARKGREEN);

	for (int i = 0; i < 4; i++) {
		BSP_LCD_SetTextColor(objects[i].color);
		BSP_LCD_FillRect(objects[i].x, objects[i].y,
				         objects[i].width, objects[i].height);
	}

	if (gameState == Crash) {
		printChar("Kraksa!");
		SwitchLayersSync();
		GameTakeOver();
	}

	printLine = 0;
}
