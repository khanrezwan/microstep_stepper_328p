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
//#include<avr/io.h>
#include<util/delay.h>
#include <avr/interrupt.h> // for interrupt isr
#include <util/atomic.h> // for interrupt atomic execution
//#include <avr/pgmspace.h> // for PROGMEM
#include "my_74hc595_driver.h" //shift register driver
#include "micro_step_tables.h"

#define DEBUG_Print
#ifdef DEBUG_Print
#include "my_usart.h"
volatile uint8_t print_flag_int0=0;
volatile uint8_t print_flag_0_ADC = 0;//print flag for channel 0
volatile uint8_t print_flag_1_ADC = 0;// print flag for channel 1
#endif

//#define L298_test //Enables 1.1V internal reference for sense resistor..@see ADC_init()


//Function declarations
void int0_init(void); //setup interrupt SFRs
void set_Step_Jump(void); //setup microsteps
void IO_PORT_Init(void); //initilize I/O port
void setup_PWM(void); //setup PWM SFRs @ 20kHz
void PoR_step(void); //on Power on Reset take one full step Forward

void timer2_init(void);//timer2 will calculate time for decay and dead time

void timer0_init(void);//timer0 will trigger ADC and

void ADC_disable();//Disable ADC
void ADC_enable();//Enable ADC
void ADC_init();//initialize ADX

//PROGMEM Store lookup table in program memory
//static const int sin_table[17] PROGMEM ={0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023};

volatile uint8_t ADC0=1;//first converting adc0 then adc1
volatile uint16_t ADC_result[2] = {0,0}; //conversion result for channel 0
//volatile uint16_t ADC_result_1 = 0; //conversion result for channel 1

volatile uint8_t Step_Number = 0;
volatile uint8_t Step_Jump = 0;
volatile uint8_t n_timer2=0;
#define FWD 1 //Clockwise
#define REV 0 //Anti- Clockwise

#define Debug_LED PB5
#define Debug_LED_DDR DDRB
#define Debug_LED_Port PORTB


/*
 * atmega328p -> H bridge
 *  */

/*
 * 7408 pin 3 -> H bridge InA/ In1 -> H bridge OutA /Out1 -> Stepper 1A
 * 7408 pin 6 -> H bridge InB/ In2 -> H bridge OutB /Out2 -> Stepper 1B
 * 7408 pin 8 -> H bridge InC/ In3 -> H bridge OutC /Out3 -> Stepper 2A
 * 7408 pin 11 -> H bridge InD/ In4 -> H bridge OutD /Out4 -> Stepper 2B
 * */

///defining bit number for shift reg
#define CNT1 7 //Phase1_A will goto 408(AND gate) pin 2.
#define NotCNT1 6 //Phase1_B will goto 408(AND gate) 7408(AND gate) pin 9.
#define CNT2 5 //Phase2_A will goto 7408(AND gate) pin 5.
#define NotCNT2 4 //Phase2_B will goto 7408(AND gate) pin 12.
#define Enable_A_B 3 //Enable H Bridge 1 i.e.. 1A and 1B Enable_A_B
#define Enable_C_D 2 //Enable H Bridge 1 i.e.. 2A and 2B Enable_C_D


///defining bits for shift reg to 74157 interface
///All four channel A's will be connected to CCP1 and CCP2
///All four channel B's will be pulled high

#define Select 1 //pin 1 of 74157 Low selects A and High Selects B
#define Strobe 0 //pin 15. Low enables outputs. High disables. optional we may connect it to Ground

/*
 * 328P -> 7408 pin 1,4,10,13; in future 74157 multiplexer
 *  */
#define CCP_PORT PORTB
#define CCP_DDR DDRB
#define CCP1 PB1 //OC0A goes to 74157
#define CCP2 PB2 //OC0B goes to 74157
#define Sin_PhaseA OCR1A //Register for 10bit PWM Phase A
#define Sin_PhaseB OCR1B //Register for 10bit PWM  Phase B
uint16_t Sin_PhaseA_variable,Sin_PhaseB_variable;
uint8_t data_to_be_Shifted;
/*
 * DipSwitch -> Atmega328P
 *  */
#define Mode_switch_DDR DDRD
#define Mode_switch_PORT PORTD
#define Mode_switch_PIN PIND
#define MS1 PD4
#define MS2 PD5
#define MS3 PD6
/**
 * H-bridge Sense resistor -> AVR
 *
 * */
#define SENSE01 PC0
#define SENSE02 PC1
/*
 * stepper driver inputs (Logic or other MCU) -> atmega328p
 *  */
#define step_dir_DDR DDRD
#define step_dir_PORT PORTD
#define step_dir_PIN PIND
#define step PD2 // external interrupt 0
#define dir PD3
//Direction FWD clockwise and REV anti-clockwise
#define Chip_Enable_DDR DDRD
#define Chip_Enable_PORT PORTD
#define Chip_Enable_PIN PIND
#define Chip_Enable PD7 // Brake functionality
/////////////////////////IO Funcionality///////////////////////////////
void IO_PORT_Init(void)
{
	Mode_switch_DDR &= ~((1<<MS1)|(1<<MS2)|(1<<MS3));
	step_dir_DDR &=~((1<<step)|(1<<dir));
	step_dir_PORT |=(1<<step); //pull up
	Chip_Enable_DDR&=~(1<<Chip_Enable);
	Debug_LED_DDR|=(1<<Debug_LED);
}
/////////////////////////////int0////////////////////////////////////////
void int0_init(void)
{
		EICRA |= (1<<ISC01); /// EICRA – External Interrupt Control Register A ; ISC01, ISC00 = 1,0 falling edge triggers interrupt
		EICRA &= ~(1<<ISC00);
		EIMSK|= (1<<INT0);/// Enable INT0
}

ISR(INT0_vect)
{
	if (Chip_Enable_PIN & (1<<Chip_Enable))//if ChipEnable then take a step
		{
		Debug_LED_Port^=(1<<Debug_LED);
			//uint8_t shift_data;
			//PWM_FSM_DRIVER(); //not calling this function to reduce status save
			Step_Number += Step_Jump;

			if (Step_Number > 127) {
				Step_Number = 0;
			}
			// if (flag == 0b00000001)


			Sin_PhaseB_variable= pgm_read_word(&sin_table_Phase_B[Step_Number]);
			Sin_PhaseA_variable= pgm_read_word(&sin_table_Phase_A[Step_Number]);

			if (step_dir_PIN & (1<<dir))//DIR= FWD
			{
				data_to_be_Shifted=pgm_read_byte(&Step_table_normal_forward[Step_Number]);
				//shift_reg_load_8_bits(shift_data);

			}
			else //DIR = REV
			{
				data_to_be_Shifted=pgm_read_byte(&Step_table_normal_reverse[Step_Number]);
				//shift_reg_load_8_bits(shift_data);
			}
			#ifdef DEBUG_Print
			print_flag_int0 =1;
			#endif

		}
}
//////////////////////////set step jump///////////////////////////////////
void set_Step_Jump(void)
/*setup microsteps*/
{
	Step_Jump = pgm_read_byte(&step_jump_table[(Mode_switch_PIN & ((1<<MS1)|(1<<MS2)|(1<<MS3)))>>4]); // Read PIND for MS1,MS2 and MS3
	#ifdef DEBUG_Print
	printf("Step_Jump= %d\n",Step_Jump);
	#endif
}
///////////////////////////////////////PWM/////////////////////////////////////
ISR(TIMER1_OVF_vect)
{
	//todo add a flag to indicate Sin_PhaseA or B updated
	Sin_PhaseA = Sin_PhaseA_variable;
	Sin_PhaseB = Sin_PhaseB_variable;
	shift_reg_load_8_bits(data_to_be_Shifted);
}
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
	TIMSK1|=(1<<TOIE1);//Enable overflow so that we may update Sin_PhaseA and B there
	ICR1 = 400;
	Sin_PhaseA_variable=399;//Set duty cycle for channel A
	Sin_PhaseA_variable=103;//Set duty cycle for channel B
	TCCR1B|=(0<<CS12)|(0<<CS11)|(1<<CS10);//CS12->10 = 001 is for 1:1 timer pre-scaler. Start timer
	CCP_DDR |= (1<<CCP1)|(1<<CCP2);
}
///////////timer2 for fast decay and dead time////////////////////
/**
 * 16MHz counts 26
 * (256/(16*10^6))S*10^6 =  16uS
 * 40KHz PWM i.e. 25uS time period
 * we r acknowledging steps at 20KHz i.e. 50uS
 * overflow at 5uS
 * so if we add pre scaler of 1:8 then overflow =16*8 = 128uS
 * for 5uS we need to count (256/128uS) *5uS  = 10 after we overflow
 * */
ISR(TIMER2_OVF_vect)
{
	//ToDo add flag variable to prevent this thing from happening before first step
	 n_timer2++;
	 if(n_timer2>10)//keep track of 50uS i.e.
		 n_timer2=0;
	 if(n_timer2==6)
	 {
		 //Dead Time
		 shift_reg_load_8_bits(Dead_time);
	 }
	 if(n_timer2>6 && n_timer2<=8)
	 {
		 //fast decay in mixed decay winding
		 //slow decay in slow decay winding
	 }
	 if(n_timer2>8 && n_timer2<=10)
	 {
		 //slow decay in mixed decay winding
		 //slow decay in slow decay winding
	 }
	 TCNT2=245;

}
void timer2_init(void)
{
	TCNT2=245;
	n_timer2=0;
	TCCR2A = 0x00; //normal operation no ouputs in oc0a or oc0b
	TIMSK2|=(1<<TOIE2); //enable Timer0 overflow
	TCCR2B |=(1<<CS21);//set prescaler 1:8 and start timer

}
//////////////////////ADC0 and ADC1 triggered by Timer0//////////////////////////////
ISR(TIMER0_OVF_vect)
{
	//may need to initate a flag for consecutive ADC conversion
}

void timer0_init(void)
{
	//TCNT0=245;
	//n_timer=0;
	TCCR0A = 0x00; //normal operation no ouputs in oc0a or oc0b
	TIMSK0|=(1<<TOIE0); //enable Timer0 overflow
	//TCCR0B |=(1<<CS00);//set prescaler 1:1 and start timer todo re calculate timer need just dumping code
	TCCR0B |=(1<<CS02)|(1<<CS00);//set prescaler 1:1024 test code
}
ISR(ADC_vect)
{
	if(ADC0)
	{
		ADC0 = 0;
		ADC_result[SENSE01] = ADC;//read the conversion
		ADMUX |=(1<<SENSE02);// 0b01000010;//change back channel to 0
		#ifdef DEBUG_Print
		print_flag_0_ADC = 1;
		#endif
		TCNT0 = 254;//set timer so that next conversion begins immediately
	}
	else
	{
		ADC0 = 1;
		ADC_result[SENSE02] = ADC;//read the conversion
		ADMUX &=~(1<<SENSE02);// 0b01000000;//change back channel to 0
		TCNT0 = 0 ;//initialize at some value
		#ifdef DEBUG_Print
		print_flag_1_ADC = 1;
		#endif
	}
	//use some flag to indicate that a valid step has been taken
}

void ADC_enable()
{
	ADCSRA |= (1<<ADEN)|(1<<ADIE);
}

void ADC_disable()
{
	ADCSRA &= ~(1<<ADEN)|~(1<<ADIE);
}

void ADC_init()
{

		#ifndef L298_test
		/// ADMUX section 23.9.1 page 262
		///BIT 7 and BIT 6 – AVCC with external capacitor at AREF pin REFS0 =0 and REFS1= 1
		/// BIT 5 – ADC Left Adjust Result ADLAR = 0
		/// BIT 3:0 –MUX3:0 0b0000 for channel 0
		ADMUX = 0b01000000;
		//same as previous line
	    //  ADMUX = (_BV(REFS1))| (ADMUX & ~_BV(REFS0))| (ADMUX & ~_BV(ADLAR))|(ADMUX & ~_BV(MUX3))|(ADMUX & ~_BV(MUX2))|(ADMUX & ~_BV(MUX1))|(ADMUX & ~_BV(MUX0));
		#endif
		#ifdef L298_test
		// We will be using 0.1ohm 5W sense resistor with L298
		// at maximum 2A the voltage drop will be 2A*0.1ohm = 0.2V
		// We will amplify that 0.2V to 1.1V using non inverting OPAMP amplifier with Rf/R1 =4.7Kohm / 47Kohm and Rg/R2 = 1Kohm or 10.5Kohm resistors
		// Hence we will be getting full 10 bit resolution using 1.1 internal Vref of 328P
		ADMUX = 0b11000000; /////BIT 7 and BIT 6 – Internal 1.1V Voltage Reference with external capacitor at AREF pin REFS0 =1 and REFS1= 1
		#endif



	    ///DIDR0 – Digital Input Disable Register 0 section Section 23.9.4 page 265 - 266
	    /// Disable digital input buffer of ADC0  and ADC1 to save power consumption
	    DIDR0 |=(1<<SENSE01)|(1<<SENSE02);
	    /// ADSCRB ADC Control and Status Register A Section 23.9.4 page 265 -266
	    /// BIT2:0 Auto Trigger Source Select 100 = Timer0 overflow
	    ADCSRB=0b00000100;
	    /// ADSCRA ADC Control and Status Register A Section 23.9.2 page 263 -264
	    ///Bit 7 – ADEN: ADC Enable =0 disable ADC
	    ///Bit 6 – ADSC: ADC Start Conversion =0 do not start conversion
	    ///Bit 5 – ADATE: ADC Auto Trigger Enable = 1 enable trigger
	    ///Bit 4 – ADIF: ADC Interrupt Flag = 0
	    ///Bit 3 – ADIE: ADC Interrupt Enable = 0 Disable ADC interrupt
	    ///Bits 2:0 – ADPS2:0: ADC Prescaler Select Bits 010 division factor = 4 todo recheck the prescaler
	    ADCSRA = 0b00100010;
}
int main()
{
	#ifdef DEBUG_Print
	USART_config(9600);//9600 for proteus simulation
	#endif
	shift_reg_init();

	shift_reg_clear_memory(1);
	shift_reg_enable_outputs();
	setup_PWM();
	IO_PORT_Init();
	int0_init();
	set_Step_Jump();
	ADC_init();
	ADC_enable();
	timer0_init();

	#ifdef DEBUG_Print
	printf("Initialized\n");
	#endif
	sei();

	while(1)
	{
		#ifdef DEBUG_Print
		if(print_flag_int0)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				print_flag_int0 =0;
			}

			printf("Step number %d Direction %d SinPhaseA %d SinPhaseB %d Shift Reg Data 0x%x\n",Step_Number,(step_dir_PIN&(1<<dir))>>dir,Sin_PhaseA_variable,Sin_PhaseB_variable,data_to_be_Shifted);
		}
		if(print_flag_0_ADC)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				print_flag_0_ADC =0;
			}
			printf("Sense01 / ADC0 value %d\n",ADC_result[SENSE01]);
		}
		if(print_flag_1_ADC)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				print_flag_1_ADC =0;
			}
			printf("Sense02 / ADC1 value %d\n",ADC_result[SENSE02]);
		}
		#endif
	}
}
