/*
 * lcd.c
 *
 *  Created on: Nov 3, 2022
 *      Author: anace
 */
#include "lcd.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>


/********************************************************************/
/*  FUNCOES CONTROLE DE DISPLAY LCD 16X2                            */
/* DEVICE: STM32F103C8						                        */
/*                                                                  */
/* INTERFACE: STM32Cube IDE								            */
/*                                                                  */
/* HARDWARE :                                             	    	*/
/* PINO DISPLAY		PINO STM32										*/
/* 	1(GND)			GND												*/
/*	2(VCC)			+5V												*/
/*	3(VO)			GND (contraste vai ficar forte!)				*/
/*	4(EN)			PB3												*/
/*	5(RW)			GND												*/
/*	6(RS)			PB4												*/
/*	7(D0)			nao ligar										*/
/*	8(D1)			nao ligar										*/
/*	9(D2)			nao ligar										*/
/*	10(D3)			nao ligar										*/
/*	11(D4)			PB5												*/
/*	12(D5)			PB6												*/
/*	13(D6)			PB7												*/
/*	14(D7)			PB8												*/
/*	15(A)			+5V ou +3.3V									*/
/*	16(K)			GND												*/
/********************************************************************/


//display LCD

#if SIMULATE

#define LCD_D0  GPIO_PIN_4		//D0 = D4
#define LCD_D1  GPIO_PIN_3		//D1 = D5
#define LCD_D2  GPIO_PIN_1		//D2 = D6
#define LCD_D3  GPIO_PIN_0		//D3 = D7
#define LCD_RS  GPIO_PIN_6
#define LCD_EN  GPIO_PIN_5

#else

#define LCD_D0  GPIO_PIN_5		//D0 = D4
#define LCD_D1  GPIO_PIN_6		//D1 = D5
#define LCD_D2  GPIO_PIN_7		//D2 = D6
#define LCD_D3  GPIO_PIN_8		//D3 = D7
#define LCD_RS  GPIO_PIN_3
#define LCD_EN  GPIO_PIN_4

#endif

#define LCD_EN_ON       HAL_GPIO_WritePin(GPIOB,LCD_EN,1)
#define LCD_EN_OFF      HAL_GPIO_WritePin(GPIOB,LCD_EN,0)
#define LCD_RS_ON       HAL_GPIO_WritePin(GPIOB,LCD_RS,1)
#define LCD_RS_OFF      HAL_GPIO_WritePin(GPIOB,LCD_RS,0)
#define LCD_D0_ON      	HAL_GPIO_WritePin(GPIOB,LCD_D0,1)
#define LCD_D0_OFF      HAL_GPIO_WritePin(GPIOB,LCD_D0,0)
#define LCD_D1_ON       HAL_GPIO_WritePin(GPIOB,LCD_D1,1)
#define LCD_D1_OFF      HAL_GPIO_WritePin(GPIOB,LCD_EN,0)
#define LCD_D2_ON       HAL_GPIO_WritePin(GPIOB,LCD_D2,1)
#define LCD_D2_OFF      HAL_GPIO_WritePin(GPIOB,LCD_EN,0)
#define LCD_D3_ON       HAL_GPIO_WritePin(GPIOB,LCD_D3,1)
#define LCD_D3_OFF      HAL_GPIO_WritePin(GPIOB,LCD_EN,0)
#define LCD_DATA_OFF    HAL_GPIO_WritePin(GPIOB,LCD_D0|LCD_D1|LCD_D2|LCD_D3,0)

/********************************************************************/

/******************************* FUNCOES ******************************/
/****************ENVIA UM COMANDO PARA O DISPLAY*********************/
void LCDCmd (unsigned char cmd)
{
  LCD_RS_OFF;
  sendnibble(cmd>>4);           //uso de 4 bits apenas
  sendnibble(cmd&0x0F);         //escreve 4 bits
}
/********************************************************************/
/***********ROTINA PARA TRABALHAR COM 4 BITS NO DISPLAY**************/
void sendnibble(unsigned char dado)
{
  LCD_DATA_OFF;
  if ((dado&0x01)==0x01) LCD_D0_ON;   //atualiza valor de dado
  if ((dado&0x02)==0x02) LCD_D1_ON;   //atualiza valor de dado
  if ((dado&0x04)==0x04) LCD_D2_ON;   //atualiza valor de dado
  if ((dado&0x08)==0x08) LCD_D3_ON;   //atualiza valor de dado
  LCD_EN_ON;
  delayLCD();
  LCD_EN_OFF;
}
/********************************************************************/
/******************INICIALIZA O DISPLAY******************************/
void InitLCD (void)
{
  LCD_EN_OFF;
  LCD_RS_OFF;
  sendnibble(0x30>>4);
  delayLCD();
  sendnibble(0x30>>4);
  delayLCD();
  sendnibble(0x30>>4);
  delayLCD();
  sendnibble(0x20>>4);
  delayLCD();
  LCDCmd(0x28);
  delayLCD();
  LCDCmd(0x08);
  delayLCD();
  LCDCmd(0x0C);
  delayLCD();
  LCDCmd(0x01);
  delayLCD();
   //Transaction end
  //dispSend(0x48, command); //Same thing, but for 0x01
  //for(i=0; i<8; i++) dispSend(cact[i], write);
  //dispSend(0x80, command);
}
/********************************************************************/
/*******************ATRASO PARA CONFIGURAR O DISPLAY*****************/
void delayLCD (void)
{
	HAL_Delay(1);
}
/********************************************************************/
/********ENVIA UMA MENSAGEM PARA O DISPLAY NA POSICAO X,Y************/
void LCDPrintXYStr (unsigned char x, unsigned char y, char *dado)
{
  unsigned char pos;
  pos=x-1;
  if (y==1)
  {
    pos=pos+0x80;
    LCDCmd(pos);
  }
  else if(y==2)
  {
    pos=pos+0xc0;
    LCDCmd(pos);
  }
  LCDPrintStr(dado);
}
/********************************************************************/
/*******************ENVIA UMA MENSAGEM AO DISPLAY********************/
void LCDPrintStr (char *dado)
{
	while (*dado != 0)
	{
		LCDChar(*dado);
		dado++;
	}
}
/********************************************************************/
/*****************ENVIA UM INTEIRO PARA O DISPLAY********************/
void LCDPrintVal (unsigned int dado)
{
  if(dado>=10000) LCDChar((dado/10000)+0x30);
  if(dado>=1000) LCDChar(((dado%10000)/1000)+0x30);
  if(dado>=100)  LCDChar((((dado%10000)%1000)/100)+0x30);
  if(dado>=10)   LCDChar(((((dado%10000)%1000)%100)/10)+0x30);
  LCDChar(((((dado%10000)%1000)%100)%10)+0x30);
}
/********************************************************************/
/*********ENVIA UM INTEIRO PARA O DISPLAY NAS POSICOES X e Y*********/
void LCDPrintXYVal (unsigned char x,unsigned char y,unsigned int dado)
{
  unsigned char pos;
  pos=x-1;
  if (y==1)
  {
    pos=pos+0x80;
    LCDCmd(pos);
  }
  else
  {
    pos=pos+0xc0;
    LCDCmd(pos);
  }
  LCDPrintVal(dado);
}

/**********************ENVIA UM DADO AO DISPLAY**********************/
void LCDChar (unsigned char dado)
{
  LCD_RS_ON;
  sendnibble(dado>>4);
  sendnibble(dado&0x0F);
}
/********************************************************************/
/**********************ENVIA UM COMANDO AO DISPLAY**********************/
void LCD(unsigned char dado)
{
  LCD_RS_ON;
  sendnibble(dado>>4);
  sendnibble(dado&0x0F);
}
/********************************************************************/
/**********************ESCOLHE MODO CURSOR **********************/
void LCDCursorMode (unsigned char modo)
{
  if(modo==0) LCDCmd(0x0c);   //apagado
  else if(modo==1) LCDCmd(0x0F);     //ligado
}
/********************************************************************/
/**********************ESCOLHE POSICAO CURSOR **********************/
void LCDCursorPos (unsigned char col,unsigned char row)
{
  unsigned char pos;
  pos=col-1;
  if (row==1)
  {
    pos=pos+0x80;
    LCDCmd(pos);
  }
  else
  {
    pos=pos+0xc0;
    LCDCmd(pos);
  }
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void createChar(uint8_t location, unsigned char charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  LCDCmd(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
	 LCDChar(charmap[i]);
  }
}

void LCDClear(void){

	LCDCmd(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
	HAL_Delay(2);
}

void RandChar(void){

	uint8_t lowerLimit = 3, upperLimit = 14;
	uint8_t col = 0, ch = 0;

	srand(HAL_GetTick()); // Seed

	col =  lowerLimit + rand() % (upperLimit - lowerLimit);
	LCDCursorPos(col, 2);

	lowerLimit = 1;
	upperLimit = 4;

	ch =  lowerLimit + rand() % (upperLimit - lowerLimit);
	LCDChar(ch);
}

void DisplayStartScreen(void) {

	char InitMsg [] = "Dino Game...";

	LCDCursorPos(1,1);
	LCDPrintStr(InitMsg);

	LCDCursorPos(1, 2);
	LCDChar(BLOCK);

	LCDCursorPos(15, 2);
	LCDChar(BLOCK);

	LCDCursorPos(2, 2);
	LCDChar(DINO);

	RandChar();
	RandChar();
	RandChar();
}

void UpdateLcd(unsigned char *runnerArea, unsigned char jump) {

  for (uint8_t i = 0; i <= 15; i++) {
	  LCDCursorPos(i, 2);
	  LCDChar(runnerArea[i]);
  }
  LCDCursorPos(1, 1);
  LCDChar(jump);

}

void PrintScore(unsigned int score){

  LCDCursorPos(4, 1);
  LCDPrintStr((char*)"Score: ");
  LCDCursorPos(11, 1);
  LCDPrintVal(score);

}

void DrawBlock(unsigned char *runnerArea){

  runnerArea[0] = BLOCK;
  runnerArea[15] = BLOCK;

}

void ShowCrashScreen(unsigned int score, unsigned int *bestScore){

	LCDCursorPos(4, 1);
	LCDPrintStr((char*)"Game Over!");
	HAL_Delay(2500);

	LCDCursorPos(4, 1);
	LCDPrintStr((char*)"Best: ");

	LCDCursorPos(10, 1);
	LCDPrintStr((char*)"      ");

	LCDCursorPos(10, 1);
 /* if (EEPROMReadInt(0) <= score) { TODO implementar leitura memória
	EEPROMWriteInt(0, score);
  }
  lcd.print(EEPROMReadInt(0));
 */
   if(score > *bestScore){
	   *bestScore = score;
   }

   LCDPrintVal(*bestScore);
}
/********************************************************************/
/********************************************************************/

