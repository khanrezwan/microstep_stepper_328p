/*
 * my_usart.h
 *
 *  Created on: Dec 16, 2016
 *      Author: rezwan
 */

#ifndef MY_USART_H_
#define MY_USART_H_
#include <avr/io.h>
#include <stdio.h>
#define FOSC 16000000 /**< Clock speed for UBRR calculation. refer page 179 of 328p datasheet. */
//#define BAUD 9600 /**< Baud Rate in bps. refer page 179 of 328p datasheet. */
//#define MYUBRR FOSC/16/BAUD-1 /**< UBRR = (F_CPU/(16*Baud))-1 for asynch USART page 179 328p datasheet. Baud rate 9600bps, assuming  16MHz clock UBRR0 becomes 0x0067*/

void USART_init(unsigned int ubrr);
int USART_send(char c, FILE *stream);
int USART_receive(FILE *stream );
void USART_config(uint16_t baud);


#endif /* MY_USART_H_ */
