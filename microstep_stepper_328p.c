/*
 * microstep_stepper_328p.c
 *
 *  Created on: Dec 12, 2016
 *      Author: rezwan
 */

/*todo 1) going to drive transistors at 40KHz PWM. T_period - 25uS
 * 		1a) Related calculation: Fast PWM Timer 1 (mode 14 fast pwm with Top icr1), 1:8 prescaler. Top = 399. Resolution 8 bit
 * todo		 2) Step going signal will be limited at 20kHz. we wont take a step if command arrives faster than that
 * 		2a) So that we may do dead time, slow/fast/mixed decay..achievable RPM at 1:32 is 187
 * 	todo 3)run one or 2 timer kernel which will keep track of step signal frequency, mix dead time and decays with 40kHz pwm
 * 	todo 4) we will need 8 outputs to control 8 transistors. we will not have 8 bits in one port. split in PortD and PortB
 *		4a) May use shift register with/without
 * 	todo 5) PD2 will be for step signal
 * 	todo 6) we will need 3 in puts for mode select
 * 	todo 7) 1 input for direction
 * 	todo 8) 2 ouput for 2 enable a.k.a pwm. Check if and gate can work at 40KHz
 * 	todo 9) 2 ADC input for sense
 * 	todo 10) 1 input for chip select...17 out of 20 usable pins gone...whoa
 * 	todo 11) need to port code written using atttiny
 * */
