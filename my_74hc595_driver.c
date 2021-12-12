/*
 * 74hc595_driver.c
 *
 *  Created on: Dec 15, 2016
 *      Author: rezwan
 */
#include "my_74hc595_driver.h"

void shift_reg_enable_outputs(void)
{
	shift_reg_OE_port&=~(1<<OE);
	shift_reg_port |=(1<<MR);
}
void shift_reg_disable_outputs(void)
{
	shift_reg_OE_port|=(1<<OE);
	shift_reg_port &= ~(1<<MR);
}

void shift_reg_clear_memory(uint8_t latch)
{
	shift_reg_OE_port &= ~(1<<OE);
	shift_reg_port &= ~(1<<MR);
	shift_reg_port|= (1<<SH_CP); /// 4. Set shift clock high
	shift_reg_port&= ~(1<<SH_CP);/// 5. Set shift clock low. The bit is then stored at i-th bit position

	if(latch)
	{
		shift_reg_latch();
	}
	shift_reg_enable_outputs();

}

/**
 * @brief Set serial clock pulse, latch clock pulse and data pins as output
 *
 */
void shift_reg_init(void)
{
    shift_reg_ddr |= (1<<SH_CP)|(1<<ST_CP)|(1<<DS)|(1<<MR);/// 1. Set as output
    shift_reg_port &= ~((1<<SH_CP)|(1<<ST_CP)|(1<<DS)|(1<<MR)); /// 2. Clear the port
    shift_reg_OE_ddr|=(1<<OE);

}
/**
 * @brief pulse the latch pin to latch data in shift register
 */
void shift_reg_latch(void)
{
    shift_reg_port|= (1<<ST_CP);/// 1. Set the latch clock pulse high
        shift_reg_port&= ~(1<<ST_CP); /// 2. Set the latch clock pulse low
}
/**
 * @brief shift 8 bits of data to a shift register.
 *
 * @details This  function accepts 8 bit data as parameter. this data is shifted to shift register one bit at a time.
 * when all data is transferred to shift register it the latches by calling appropriate function.
 *
 * @param data 8 bit data to be transferred to 74HC595
 */
void shift_reg_load_8_bits(uint8_t data)
{

    for(uint8_t i=0;i<8;i++) /// 1. Loop for each bit of a byte
    {
        if(data & 0b00000001)/// 2. Perform bitwise and with data to filter out all the bit except the 0th bit.MSB is first.
        {
            shift_reg_port|= (1<<DS);/// 2a. If the result is 1 then the value of 0th bit is 1 and set DS = 1
        }
        else
        {
            shift_reg_port &= ~(1<<DS); /// 2b. If the result is 0 then the value of 0th bit is 0 and set DS = 0
        }
        data = data>>1; /// 3. Right shift the data by 1 position to get next bit to 0th position
        shift_reg_port|= (1<<SH_CP); /// 4. Set shift clock high
        shift_reg_port&= ~(1<<SH_CP);/// 5. Set shift clock low. The bit is then stored at i-th bit position

    }
    shift_reg_latch(); /// 6. All the bits are shifted now pulse latch clock.

}

void shift_reg_load_16_bits(uint16_t data)
{

    for(uint8_t i=0;i<16;i++) /// 1. Loop for each bit of a byte
    {
        if(data & 0x0001)/// 2. Perform bitwise and with data to filter out all the bit except the 0th bit.MSB is first.
        {
            shift_reg_port|= (1<<DS);/// 2a. If the result is 1 then the value of 0th bit is 1 and set DS = 1
        }
        else
        {
            shift_reg_port &= ~(1<<DS); /// 2b. If the result is 0 then the value of 0th bit is 0 and set DS = 0
        }
        data = data>>1; /// 3. Right shift the data by 1 position to get next bit to 0th position
        shift_reg_port|= (1<<SH_CP); /// 4. Set shift clock high
        shift_reg_port&= ~(1<<SH_CP);/// 5. Set shift clock low. The bit is then stored at i-th bit position

    }
    shift_reg_latch(); /// 6. All the bits are shifted now pulse latch clock.

}
