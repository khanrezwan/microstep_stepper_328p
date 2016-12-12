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
#include<avr/io.h>
#include<util/delay.h>
#include <avr/interrupt.h> // for interrupt isr
#include <util/atomic.h> // for interrupt atomic execution
#include <avr/pgmspace.h> // for PROGMEM
//Function declarations
void setup_int(void); //setup interrupt SFRs
void set_Step_Jump(void); //setup microsteps
void IO_PORT_Init(void); //initilize I/O port
void setup_PWM(void); //setup PWM SFRs @ 20kHz
void PoR_step(void); //on Power on Reset take one full step Forward

//PROGMEM Store lookup table in program memory
//static const int sin_table[17] PROGMEM ={0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023};
static const int sin_table_Phase_A[128] PROGMEM ={399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,365,334,304,274,256,238,219,200,182,163,143,124,103,81,51,0,51,81,103,124,143,163,182,200,219,238,256,274,304,334,365,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,365,334,304,274,256,238,219,200,182,163,143,124,103,81,51,0,51,81,103,124,143,163,182,200,219,238,256,274,304,334,365,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399};;
static const int sin_table_Phase_B[128] PROGMEM={0,51,81,103,124,143,163,182,200,219,238,256,274,304,334,365,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,365,334,304,274,256,238,219,200,182,163,143,124,103,81,51,0,51,81,103,124,143,163,182,200,219,238,256,274,304,334,365,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,399,365,334,304,274,256,238,219,200,182,163,143,124,103,81,51};
static const unsigned char PORTBF[128] PROGMEM={0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010};
static const unsigned char PORTBR[128] PROGMEM={0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10000010,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b10100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01100000,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010,0b01000010};
static const unsigned char step_jump_table[8] PROGMEM = {1, 16, 8, 4, 0, 0, 0, 2};
unsigned char Step_Number = 0;
unsigned char Step_Jump = 0;
#define FWD 1 //Clockwise
#define REV 0 //Anti- Clockwise

/*
 * Attiny2313 -> L6506
 *  */

#define PHASE1_A PB1 //CNT1 or PHASE1-1 or Motor winding 1A. Goes to L6506 IN1
#define PHASE2_A PB6 //CNT2 or PHASE2-1 or Motor winding 2A. Goes to L6506 IN3
#define PHASE1_B PB5 //NotCNT1 or PHASE1-2 or Motor winding 1B. Goes to L6506 IN2
#define PHASE2_B PB7 //Not CNT2 or PHASE2-2 or Motor winding 2B. Goes to L6506 IN4
#define Ref1 PB3 // DAC /PWM output goes to L6506 voltage Ref1 pin
#define Ref2 PB4 // DAC /PWM output goes to L6506 voltage Ref2 pin
#define Sin_PhaseA OCR1A //Register for 10bit PWM Phase A
#define Sin_PhaseB OCR1B //Register for 10bit PWM  Phase B
/*
 * 328P -> L298 / L6202
 *  */
#define Enable_PORT PORTB
#define Enable_DDR DDRB
#define EN1 PB1 //OC0A if enable chopping is used in future
#define EN2 PB2 //OC0B if enable chopping is used in future

/*
 * DipSwitch -> Attiny2313
 *  */
#define MS1 PD0
#define MS2 PD1
#define MS3 PD2

/*
 * Attiny2313 Driver (Logic or other MCU) -> Attiny2313
 *  */
#define step PD3 // external interrupt
#define dir PD4 //Direction FWD clockwise and REV anti-clockwise
#define Chip_Enable PD6 // Brake functionality

void setup_PWM(void)
/*setup PWM SFRs @ 40.00kHz
 * Setup 8bit Fast PWM Mode 14 top in ICR1
 * WGM13=0, WGM12=1, WGM11=1, WGM10= 1 is for fastPWM in 10bit mode.
 * Here TOP= 0x03FF or 1023
 *  *
 * COM1A1:COM1A0 =10 COM1B1:COM1B0 =10 and for Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at TOP
 *
 * PWM_Frequency= F_osc / (PreScaler*(1+TOP))= 16MHz / (1*(399+1))=  40000 = 40 KHz
 * Resolution= log(TOP+1)/log(2)= 8 bit
 * */
{

	TCCR1B|=(1<<WGM13)|(1<<WGM12);
	TCCR1A&=~(1<<WGM10);
	TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM11);
	ICR1 = 400;
	Sin_PhaseA=399;//Set duty cycle for channel A
	Sin_PhaseB=103;//Set duty cycle for channel B
	TCCR1B|=(0<<CS12)|(0<<CS11)|(1<<CS10);//CS12->10 = 001 is for 1:1 timer pre-scaler. Start timer
	Enable_DDR |= (1<<EN1)|(1<<EN2);
}

int main()
{
	setup_PWM();

	while(1)
	{
		for(uint8_t i=0;i<128;i++)
		{
			Sin_PhaseB= pgm_read_word(&sin_table_Phase_B[i]);
			Sin_PhaseA= pgm_read_word(&sin_table_Phase_A[i]);
			_delay_ms(10);
		}
	}
}
