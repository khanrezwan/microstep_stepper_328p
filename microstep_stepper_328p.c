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
#include "my_74hc595_driver.h" //shift register driver
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
 * atmega328p -> H bridge
 *  */

/*
 * 7408 pin 3 -> H bridge InA/ In1 -> H bridge OutA /Out1 -> Stepper 1A
 * 7408 pin 6 -> H bridge InB/ In2 -> H bridge OutB /Out2 -> Stepper 1B
 * 7408 pin 8 -> H bridge InC/ In3 -> H bridge OutC /Out3 -> Stepper 2A
 * 7408 pin 11 -> H bridge InD/ In4 -> H bridge OutD /Out4 -> Stepper 2B
 * */
#define PHASE1_A PB1 //CNT1 or PHASE1-1 or Motor winding 1A. 7408(AND gate) pin 2. in future driven using Shift reg
#define PHASE2_A PB6 //CNT2 or PHASE2-1 or Motor winding 2A. 7408(AND gate) pin 9.
#define PHASE1_B PB5 //NotCNT1 or PHASE1-2 or Motor winding 1B. 7408(AND gate) pin 5.
#define PHASE2_B PB7 //Not CNT2 or PHASE2-2 or Motor winding 2B. 7408(AND gate) pin 12.
#define Ref1 PB3 // DAC /PWM output goes to L6506 voltage Ref1 pin
#define Ref2 PB4 // DAC /PWM output goes to L6506 voltage Ref2 pin

///defining bit number for shift reg
#define CNT1 7 //Phase1_A will goto 408(AND gate) pin 2.
#define NotCNT1 6 //Phase1_B will goto 408(AND gate) 7408(AND gate) pin 9.
#define CNT2 5 //Phase2_A will goto 7408(AND gate) pin 5.
#define NotCNT2 4 //Phase2_B will goto 7408(AND gate) pin 12.
#define Enable_A_B 3 //Enable H Bridge 1 i.e.. 1A and 1B Enable_A_B
#define Enable_C_D 2 //Enable H Bridge 1 i.e.. 2A and 2B Enable_C_D

#define Fast_Decay_A_B_Forward
#define Fast_Decay_A_B_Reverse
#define Slow_decay_AB

#define Fast_Decay_C_D_Forward
#define Fast_Decay_C_D_Reverse
#define Slow_decay_CD

///defining bits for shift reg to 74157 interface
///All four channel A's will be connected to CCP1 and CCP2
///All four channel B's will be pulled high

#define Select 1 //pin 1 of 74157 Low selects A and High Slects B
#define Strobe 0 //pin 15. Low enables outputs. High disables. optional we may connect it to Ground

/*
 * 328P -> 7408 pin 1,4,10,13; in future 74157 multiplexer
 *  */
#define CCP_PORT PORTB
#define CCP_DDR DDRB
#define CCP1 PB1 //OC0A goes to 7408(AND gate) pin 1 and 4
#define CCP2 PB2 //OC0B goes to 7408 (AND gate) pin 10 and 13
#define Sin_PhaseA OCR1A //Register for 10bit PWM Phase A
#define Sin_PhaseB OCR1B //Register for 10bit PWM  Phase B
/*
 * DipSwitch -> Atmega328P
 *  */
#define MS1 PD0
#define MS2 PD1
#define MS3 PD2

/*
 * stepper driver inputs (Logic or other MCU) -> atmega328p
 *  */
#define step PD3 // external interrupt
#define dir PD4 //Direction FWD clockwise and REV anti-clockwise
#define Chip_Enable PD6 // Brake functionality

///////////////////////////////////////PWM/////////////////////////////////////
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
	CCP_DDR |= (1<<CCP1)|(1<<CCP2);
}

int main()
{
	setup_PWM();

	shift_reg_init();
	//shift_reg_clear_memory(1);
	shift_reg_enable_outputs();
//	shift_reg_load_16_bits(spi_data);

	while(1)
	{
		for(uint8_t i=0;i<128;i++)
		{
			Sin_PhaseB= pgm_read_word(&sin_table_Phase_B[i]);
			Sin_PhaseA= pgm_read_word(&sin_table_Phase_A[i]);

			shift_reg_load_8_bits(0xF0);
			_delay_ms(10);
			shift_reg_clear_memory(1);
			_delay_ms(10);
		}
	}
}
