/*
 * 74hc595_driver.h
 *
 *  Created on: Dec 15, 2016
 *      Author: rezwan
 */

#ifndef MY_74HC595_DRIVER_H_
#define MY_74HC595_DRIVER_H_
#include <avr/io.h>
#define shift_reg_port PORTC /**< Port register for Shift Register */
#define shift_reg_ddr DDRC /**< Data direction register for Shift Register */
#define shift_reg_OE_port PORTB /**< Port register for Shift Register */
#define shift_reg_OE_ddr DDRB /**< Data direction register for Shift Register */
#define SH_CP PC2 /**< Shift register Serial clock pulse (PIN 11 of 74HC595) connection to MCU I/O Port bit. */
#define ST_CP PC3 /**< Shift register Latch clock pulse (PIN 12 of 74HC595) connection to MCU I/O Port bit. */
#define DS PC5 /**< Shift register Data (PIN 14 of 74HC595) connection to MCU I/O Port bit. */
#define OE PB4 /**< Shift register Output Enable (PIN 13 of 74HC595) A high disable outputs to High impedance */
#define MR PC4 /**< Shift register Output Enable (PIN 15 of 74HC595) master reset active low */
void shift_reg_init(void);
void shift_reg_latch(void);
void shift_reg_enable_outputs(void);
void shift_reg_disable_outputs(void);
void shift_reg_clear_memory(uint8_t latch);/**< latch 0 clears the shift reg, latch > 0 clears the memory and output*/
void shift_reg_load_8_bits(uint8_t data);
void shift_reg_load_16_bits(uint16_t data);

#endif /* 74HC595_DRIVER_H_ */
