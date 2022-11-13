/*
 * lcd.h
 *
 *  Created on: Nov 3, 2022
 *      Author: anace
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#include <stdint.h>


#define SIMULATE (0)

#define LCD_SETCGRAMADDR 0x40
#define LCD_CLEARDISPLAY 0x01

#define DINO 	(0u)
#define CACTUS 	(1u)
#define BIRD	(2u)
#define BLOCK 	(3u)
#define SPACE	(32u) //Prints nothing
//Declaracao das funcoes do LCD

/********************************************************************/
void LCDCmd (unsigned char cmd);
void InitLCD (void);
void LCDPrintXYStr (unsigned char x, unsigned char y, char *dado);
void LCDPrintStr ( char *dado);
void LCDPrintVal (unsigned int dado);
void LCDCursorPos (unsigned char, unsigned char );
void LCDCursorMode (unsigned char modo);
void LCDPrintXYVal (unsigned char x, unsigned char y,unsigned int dado);
void sendnibble(unsigned char dado);
void LCDChar (unsigned char dado);
void delayLCD (void);
void createChar(uint8_t, unsigned char[]);
void LCDClear(void);
void RandChar(void);
void DisplayStartScreen(void);
void UpdateLcd(unsigned char*, unsigned char);
void PrintScore(unsigned int);
void DrawBlock(unsigned char*);
void ShowCrashScreen(unsigned int, unsigned int*);




#endif /* SRC_LCD_H_ */
