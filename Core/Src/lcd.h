/*
 * lcd.h
 *
 *  Created on: Nov 3, 2022
 *      Author: anace
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

//Declaracao das funcoes do LCD

#define SIMULATE (1)

/********************************************************************/
void LCDCmd (unsigned char cmd);
void InitLCD (void);
void LCDPrintXYStr (unsigned char x, unsigned char y, char *dado);
void LCDPrintStr ( char *dado);
void LCDPrintVal (unsigned int dado);
void LCDCursorPos (unsigned char x, unsigned char y);
void LCDCursorMode (unsigned char modo);
void LCDPrintXYVal (unsigned char x, unsigned char y,unsigned int dado);
void sendnibble(unsigned char dado);
void LCDChar (unsigned char dado);
void delayLCD (void);


#endif /* SRC_LCD_H_ */
