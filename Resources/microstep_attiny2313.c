/*
 * microstep_attiny2313.c
 *
 *  Created on: Aug 25, 2013
 *      Author: rezwan
 */

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


//void PWM_fsm(void); //For State 0->127

//PROGMEM Store lookup table in program memory
//static const int sin_table[17] PROGMEM ={0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023};
static const int sin_table_Phase_A[128] PROGMEM ={1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,938,858,781,705,658,611,563,515,467,418,369,320,265,210,131,0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,938,858,781,705,658,611,563,515,467,418,369,320,265,210,131,0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023};
static const int sin_table_Phase_B[128] PROGMEM ={0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,938,858,781,705,658,611,563,515,467,418,369,320,265,210,131,0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,938,858,781,705,658,611,563,515,467,418,369,320,265,210,131};
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
 * Attiny2313 -> L298 / L6202
 *  */
#define EN1 PB2 //OC0A if enable chopping is used in future
#define EN2 PD5 //OC0B if enable chopping is used in future

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

//EIFR and GIMSK was not detected by eclipse IDE it was trying to use GIFR and GICR instead
// Compiler was giving erro so manually overridden
#define EIFR    _SFR_IO8(0x3A)
#define INTF2   5
#define INTF0   6
#define INTF1   7

#define GIMSK    _SFR_IO8(0x3B)
#define PCIE    5
#define INT0    6
#define INT1    7

ISR(INT1_vect)
{
	if (PIND & 0b01000000)//if ChipEnable then take a step
	{
		//PWM_FSM_DRIVER(); //not calling this function to reduce status save
		Step_Number += Step_Jump;

		if (Step_Number > 127) {
			Step_Number = 0;
		}
		// if (flag == 0b00000001)
		unsigned char temp;

		Sin_PhaseB= pgm_read_word(&sin_table_Phase_B[Step_Number]);
		Sin_PhaseA= pgm_read_word(&sin_table_Phase_A[Step_Number]);

		if (PIND &0b00010000)//DIR= FWD
		{
			temp= PORTB & 0b00011101;
			temp|=pgm_read_byte(&PORTBF[Step_Number]);
			PORTB=temp;
			//PORTB
			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
		} else {
			temp= PORTB & 0b00011101;
			temp|=pgm_read_byte(&PORTBR[Step_Number]);
			PORTB=temp;
			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
		}

	}
}

void setup_int(void)
/*setup interrupt SFRs*/
{
	DDRD|=(0<<step); // input
	PORTD|=(1<<step); //Enable pull up //check

	GIMSK|=(1<<INT1);//Unmask INT1 @ PD3
	EIFR|=(1<<INTF1);
	MCUCR|=(1<<ISC11)|(0<<ISC10); //The falling edge of INT1 generates an interrupt request. Table 31 Pg60
	sei(); //Set Global Interrupt
}
void set_Step_Jump(void)
/*setup microsteps*/
{
	Step_Jump = pgm_read_byte(&step_jump_table[PIND & 0b00000111]); // Read PIND for MS1,MS2 and MS3
}
void IO_PORT_Init(void)
/*initilize I/O port*/
{
	DDRB|=(1<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(1<<PHASE2_B)|(1<<EN1);//PortB I/Os
	DDRD|=(1<<EN2)|(0<<MS1)|(0<<MS2)|(0<<MS2)|(0<<dir)|(0<<Chip_Enable);//PortD I/Os
}
void setup_PWM(void)
/*setup PWM SFRs @ 19.53kHz
 * Setup 10bit Fast PWM Mode 7
 * WGM13=0, WGM12=1, WGM11=1, WGM10= 1 is for fastPWM in 10bit mode.
 * Here TOP= 0x03FF or 1023
 * For more information 2313 datasheet table 46 pg 106
 *
 * COM1A1:COM1A0 =10 COM1B1:COM1B0 =10 and for Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at TOP
 * For more information 2313 datasheet table 44 pg 104
 * PWM_Frequency= F_osc / (PreScaler*(1+TOP))= 20MHz / (1*(1023+1))=  19531.25 = 19.5 KHz
 * Resolution= log(TOP+1)/log(2)= 10 bit
 * */
{
	DDRB|=(1<<Ref2)|(1<<Ref1);//OCR1A and OCR1B as output
	TCCR1B|=(0<<WGM13)|(1<<WGM12);
	TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM11)|(1<<WGM10);
	Sin_PhaseA=0x0398;//Set duty cycle for channel A
	Sin_PhaseB=0x0102;//Set duty cycle for channel B
	TCCR1B|=(0<<CS12)|(0<<CS11)|(1<<CS10);//CS12->10 = 001 is for 1:1 timer pre-scaler. Start timer

}
void PoR_step(void)
/*on Power on Reset take one full step Forward*/
{

}

//void PWM_fsm(void)
///*
// * There are maximum 128 states for 1/32 micro stepping
// * Calculation 1 full step is divided into 32 micro steps
// * 4 full steps completes a sequence
// * Hence 4*32= 128 states
// *
// *
// */
//{
//	switch (Step_Number)
//	{
//
//	/*
//	     //0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
//	    PHASE2_A  = RB2 PHASE1_A =  RB3P HASE2_B = RB4 PHASE1_B = RB5
//
//
//	 */
//	case 0:
//
//		Sin_PhaseB=0;
//		//CCPR2L = 0x0;
//		//CCP2CON = 0x0C;
//		if (PIND &0b00010000)//DIR= FWD
//		{
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//			//PORTB =
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//		} else {
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//		}
//		break;
//		/*
//	              	     //0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
//	              	    PHASE2_A  = RB2 PHASE1_A =  RB3
//	              	    PHASE2_B = RB4 PHASE1_B = RB5
//
//
//		 */
//	case 1:
//
//		Sin_PhaseB=131;
//		if (PIND &0b00010000)//DIR= FWD
//		{
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//		} else {
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//		}
//		break;
//		/*
//			              	     //0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
//			              	    PHASE2_A  = RB2 PHASE1_A =  RB3
//			              	    PHASE2_B = RB4 PHASE1_B = RB5
//
//
//		 */
//	case 2:
//
//		// CCPR2L = 0x34;
//		//CCP2CON = 0x2C;
//		Sin_PhaseB=210;
//		if (PIND &0b00010000)//DIR= FWD
//		{
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//			// PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//		} else {
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//		}
//
//		break;
///*
////0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
// PHASE2_A  = RB2 PHASE1_A =  RB3
//PHASE2_B = RB4 PHASE1_B = RB5		 */
//	case 3:
//		Sin_PhaseB=265;
//		//CCPR2L = 0x42;
//		//CCP2CON = 0x1C;
//		if (PIND &0b00010000)//DIR= FWD
//		{			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//		//  PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//		} else {
//			PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//			//PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//		}
//		break;
//		/*
//		//0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
//		 PHASE2_A  = RB2 PHASE1_A =  RB3
//		PHASE2_B = RB4 PHASE1_B = RB5		 */
//	 case 4:
//		 	 	Sin_PhaseB=320;
//	         //   CCPR2L = 0x50;
//	           // CCP2CON = 0x0C;
//		 	 	if (PIND &0b00010000) {
//		 	 		PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//	               // PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//	            } else {
//	            	PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//	               // PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//	            }
//	            break;
//	            /*
//	            		//0,131,210,265,320,369,418,467,515,563,611,658,705,781,858,938,1023
//	            		 PHASE2_A  = RB2 PHASE1_A =  RB3
//	            		PHASE2_B = RB4 PHASE1_B = RB5		 */
//	 case 5:
//		 	 	 Sin_PhaseB=369;
//	             //CCPR2L = 0x5C;
//	             //CCP2CON = 0x1C;
//		 	 	if (PIND &0b00010000) {//FWD
//		 	 		PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//	                 //PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//	             } else {
//	            	 PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//	                // PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//	             }
//	             break;
//	        case 6:
//
//           // CCPR2L = 0x68;
//          //  CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 7:
//
//           // CCPR2L = 0x74;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 8:
//
//
//          //  CCPR2L = 0x80;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 9:
//
//
//          //  CCPR2L = 0x8C;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 10:
//
//
//           // CCPR2L = 0x98;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 11:
//
//
//           // CCPR2L = 0xA4;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 12:
//
//
//           // CCPR2L = 0xB0;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 13:
//
//
//          //  CCPR2L = 0xC3;
//          //  CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 14:
//
//
//           // CCPR2L = 0xD6;
//         //   CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 15:
//
//
//           // CCPR2L = 0xEA;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 16:
//
//
//           // CCPR2L = 0xFF;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 17:
//           // CCPR1L = 0xEA;
//           // CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 18:
//            //CCPR1L = 0xD6;
//            //CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 19:
//           // CCPR1L = 0xC3;
//           // CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 20:
//            //CCPR1L = 0xB0;
//            //CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 21:
//           // CCPR1L = 0xA4;
//           // CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 22:
//            //CCPR1L = 0x98;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 23:
//           // CCPR1L = 0x8C;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 24:
//           // CCPR1L = 0x80;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 25:
//            //CCPR1L = 0x74;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 26:
//            //CCPR1L = 0x68;
//            //CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 27:
//           // CCPR1L = 0x5C;
//           // CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 28:
//            //CCPR1L = 0x50;
//            //CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 29:
//           // CCPR1L = 0x42;
//           // CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 30:
//          //  CCPR1L = 0x34;
//          //  CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 31:
//            //CCPR1L = 0x20;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 32:
//           // CCPR1L = 0x0;
//           // CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 33:
//           // CCPR1L = 0x20;
//          //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//            PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            //    PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//            PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//                //PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 34:
//            //CCPR1L = 0x34;
//            //CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 35:
//           // CCPR1L = 0x42;
//            //CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 36:
//            //CCPR1L = 0x50;
//           // CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 37:
//            //CCPR1L = 0x5C;
//           // CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 38:
//           // CCPR1L = 0x68;
//           // CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 39:
//           // CCPR1L = 0x74;
//          //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 40:
//           // CCPR1L = 0x80;
//            //CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 41:
//           // CCPR1L = 0x8C;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 42:
//          //  CCPR1L = 0x98;
//        //    CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            // flag = 0b00000010;
//            break;
//
//            // }
//            //}
//
//            //void fsm2(void) {
//            //  switch (Step_Number) {
//        case 43:
//           // CCPR1L = 0xA4;
//            //CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 44:
//           // CCPR1L = 0xB0;
//            //CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 45:
//           // CCPR1L = 0xC3;
//           // CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 46:
//           // CCPR1L = 0xD6;
//           // CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 47:
//           // CCPR1L = 0xEA;
//           // CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 48:
//           // CCPR1L = 0xFF;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 49:
//
//
//            //CCPR2L = 0xEA;
//            //CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 50:
//
//
//           // CCPR2L = 0xD6;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 51:
//
//
//            //CCPR2L = 0xC3;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 52:
//
//
//            //CCPR2L = 0xB0;
//            //CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 53:
//
//
//           // CCPR2L = 0xA4;
//            //CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 54:
//
//
//            //CCPR2L = 0x98;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 55:
//
//
//            //CCPR2L = 0x8C;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 56:
//
//
//            //CCPR2L = 0x80;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 57:
//
//
//           // CCPR2L = 0x74;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 58:
//
//
//           // CCPR2L = 0x68;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 59:
//
//
//           // CCPR2L = 0x5C;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 60:
//
//
//            //CCPR2L = 0x50;
//           // CCP2CON = 0x0C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 61:
//
//
//            //CCPR2L = 0x42;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 62:
//
//
//            //CCPR2L = 0x34;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 63:
//
//
//            //CCPR2L = 0x20;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 64:
//
//
//            //CCPR2L = 0x0;
//            //CCP2CON = 0x0C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            }
//            break;
//        case 65:
//
//
//           // CCPR2L = 0x20;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 66:
//
//
//            //CCPR2L = 0x34;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 67:
//
//
//            //CCPR2L = 0x42;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 68:
//
//
//            //CCPR2L = 0x50;
//            //CCP2CON = 0x0C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 69:
//
//
//            //CCPR2L = 0x5C;
//            //CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 70:
//
//
//           // CCPR2L = 0x68;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 71:
//
//
//           // CCPR2L = 0x74;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 72:
//
//
//            //CCPR2L = 0x80;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 73:
//
//
//            //CCPR2L = 0x8C;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 74:
//
//
//           // CCPR2L = 0x98;
//           // CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 75:
//
//
//           // CCPR2L = 0xA4;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 76:
//
//
//            //CCPR2L = 0xB0;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 77:
//
//
//           // CCPR2L = 0xC3;
//           // CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 78:
//
//
//            //CCPR2L = 0xD6;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 79:
//
//
//           // CCPR2L = 0xEA;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 80:
//
//
//            //CCPR2L = 0xFF;
//            //CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 81:
//            //CCPR1L = 0xEA;
//            //CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 82:
//       //     CCPR1L = 0xD6;
//         //   CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 83:
//       //     CCPR1L = 0xC3;
//           // CCP1CON = 0x1C;
////
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 84:
//     //       CCPR1L = 0xB0;
//     //       CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 85:
//      //      CCPR1L = 0xA4;
//     //       CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            //flag = 0b00000100;
//            break;
//
//
//            //    }
//
//            //}
//
//            //void fsm3(void) {
//            //  switch (Step_Number) {
//        case 86:
//        //    CCPR1L = 0x98;
//       //     CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 87:
//          //  CCPR1L = 0x8C;
//           // CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 88:
//        //    CCPR1L = 0x80;
//        //    CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 89:
//          //  CCPR1L = 0x74;
//        //    CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 90:
//        //    CCPR1L = 0x68;
//     //       CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 91:
//           // CCPR1L = 0x5C;
//         //   CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 92:
//         //   CCPR1L = 0x50;
//         //   CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 93:
//          //  CCPR1L = 0x42;
//         //   CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 94:
//         //   CCPR1L = 0x34;
//       //    CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 95:
//          //  CCPR1L = 0x20;
//         //   CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 96:
//         //   CCPR1L = 0x0;
//         //   CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(0<<PHASE1_A)|(1<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 97:
//        //    CCPR1L = 0x20;
//         //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 98:
//          //  CCPR1L = 0x34;
//         //   CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 99:
//         //   CCPR1L = 0x42;
//         //   CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 100:
//         //   CCPR1L = 0x50;
//          //  CCP1CON = 0x0C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 101:
//         //   CCPR1L = 0x5C;
//       //     CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 102:
//         //   CCPR1L = 0x68;
//          //  CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 103:
//         //   CCPR1L = 0x74;
//         //   CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 104:
//          //  CCPR1L = 0x80;
//        //    CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 105:
//         //   CCPR1L = 0x8C;
//          //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 106:
//        //    CCPR1L = 0x98;
//          //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 107:
//          //  CCPR1L = 0xA4;
//        //    CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 108:
//         //   CCPR1L = 0xB0;
//          //  CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 109:
//          //  CCPR1L = 0xC3;
//          //  CCP1CON = 0x1C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 110:
//         //   CCPR1L = 0xD6;
//        //    CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 111:
//           // CCPR1L = 0xEA;
//          //  CCP1CON = 0x2C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 112:
//          //  CCPR1L = 0xFF;
//          //  CCP1CON = 0x3C;
//
//
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 113:
//
//
//          //  CCPR2L = 0xEA;
//         //   CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 114:
//
//
//         //   CCPR2L = 0xD6;
//          //  CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 115:
//
//
//         //   CCPR2L = 0xC3;
//         //   CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 116:
//
//
//       //     CCPR2L = 0xB0;
//        //    CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 117:
//
//
//          //  CCPR2L = 0xA4;
//           // CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 118:
//
//
//          //  CCPR2L = 0x98;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 119:
//
//
//          //  CCPR2L = 0x8C;
//         //   CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 120:
//
//
//          //  CCPR2L = 0x80;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 121:
//
//
//           // CCPR2L = 0x74;
//        //    CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 122:
//
//
//         //   CCPR2L = 0x68;
//         //   CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 123:
//
//
//         //   CCPR2L = 0x5C;
//         //   CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 124:
//
//
//         //   CCPR2L = 0x50;
//       //     CCP2CON = 0x0C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 125:
//
//
//         //   CCPR2L = 0x42;
//         //   CCP2CON = 0x1C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 126:
//
//
//         //   CCPR2L = 0x34;
//         //   CCP2CON = 0x2C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            break;
//        case 127:
//
//
//         //   CCPR2L = 0x20;
//          //  CCP2CON = 0x3C;
//           if (PIND &0b00010000) {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(0<<PHASE2_A)|(1<<PHASE2_B);
//            } else {
//                PORTB|=(1<<PHASE1_A)|(0<<PHASE1_B)|(1<<PHASE2_A)|(0<<PHASE2_B);
//            }
//            // flag = 0b00000001;
//            break;
//    }
//
//}

int main(void)
{

	IO_PORT_Init();
	set_Step_Jump();
	setup_PWM();
	PoR_step();
	setup_int();
	//PORTD|=(FWD<<dir);
	// dir = FWD; //forward by default
	while (1) {

	}
}
