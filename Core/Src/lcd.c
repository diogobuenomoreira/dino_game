/*
 * lcd.c
 *
 *  Created on: Nov 3, 2022
 *      Author: anace
 */
#include "lcd.h"
#include "stm32f1xx_hal.h"

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
#define LCD_D0  GPIO_PIN_5		//D0 = D4
#define LCD_D1  GPIO_PIN_6		//D1 = D5
#define LCD_D2  GPIO_PIN_7		//D2 = D6
#define LCD_D3  GPIO_PIN_8		//D3 = D7
#define LCD_RS  GPIO_PIN_3
#define LCD_EN  GPIO_PIN_4
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

/*******************************FUNCOES******************************/
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
void LCD (unsigned char dado)
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
void LCDCursorPos (unsigned char x,unsigned char y)
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
}
/********************************************************************/
/********************************************************************/

