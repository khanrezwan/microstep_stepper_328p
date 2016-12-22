/*
 * micro_step_tables.h
 *
 *  Created on: Dec 16, 2016
 *      Author: rezwan
 */

#ifndef MICRO_STEP_TABLES_H_
#define MICRO_STEP_TABLES_H_
#include <avr/pgmspace.h>
#define Slow_decay 0b00001110
#define Dead_time 0b00000010
static const uint16_t sin_table_Phase_A[128] PROGMEM ={
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,365,334,304,274,256,238,219,
		200,182,163,143,124,103,81,51,
		0,51,81,103,124,143,163,182,
		200,219,238,256,274,304,334,
		365,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,365,334,304,274,256,238,
		219,200,182,163,143,124,103,81,
		51,0,51,81,103,124,143,163,
		182,200,219,238,256,274,304,334,
		365,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,399};;
static const uint16_t sin_table_Phase_B[128] PROGMEM={
		0,51,81,103,124,143,163,182,
		200,219,238,256,274,304,334,365,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,365,334,304,274,256,238,219,
		200,182,163,143,124,103,81,51,
		0,51,81,103,124,143,163,182,
		200,219,238,256,274,304,334,365,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,399,399,399,399,399,399,399,
		399,365,334,304,274,256,238,219,
		200,182,163,143,124,103,81,51};
static const uint8_t Step_table_normal_forward[128] PROGMEM={
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100};
static const uint8_t Step_table_normal_reverse[128] PROGMEM={
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,0b10011100,
		0b10011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,0b01011100,
		0b01011100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,0b01101100,
		0b01101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,
		0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100,0b10101100};


static const uint8_t step_jump_table[8] PROGMEM = {1, 16, 8, 4, 0, 0, 0, 2};

//typedef enum
//{
//	Fast, Slow, Mixed
//}decay_modes;
#define Mixed 0
#define Slow 1
#define Fast 2
const uint8_t decay_table_Sin_PhaseA[128] PROGMEM={
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow};//CCP1
//const decay_modes decay_table_Sin_PhaseA_reverse[128] PROGMEM={
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,};//CCP1
const uint8_t decay_table_Sin_PhaseB[128] PROGMEM={
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
		Slow,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed};//CCP2
//const decay_modes decay_table_Sin_PhaseB_reverse[128] PROGMEM={
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Slow,Slow,Slow,Slow,Slow,Slow,Slow,Slow,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,
//		Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed,Mixed};//CCP2
#endif /* MICRO_STEP_TABLES_H_ */
