#include <msp430.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "mc_inits.h"
#include "i2c_ctr.h"

/**
 * main.c
 */

uint16_t delay_counter = 0;
uint8_t data_transmit[16];
uint8_t *PTxData; // Pointer to TX data
uint8_t TXByteCtr;
uint8_t *PRxData; // Pointer to RX data
uint8_t RXByteCtr;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //init
    init_clocks();
    init_timers();
    init_gpios();

    i2c_init();

    _enable_interrupt();

    while(1){
        data_transmit[0] = 0x0B;
        data_transmit[1] = 0x07;
        data_transmit[2] = 0x07;
        I2C_send(0x10, data_transmit, 3);
        delay_ms(1000);
    }

    return;
}

void delay_ms(uint8_t temps){
    /*
     *
     *aquesta funció genera un delay a partir d'un timer
    */

    TB0CCR0 = 16000*temps;   // temps en ms
    while(delay_counter < temps){
        continue;
    }

    delay_counter = 0;
    return;
}


#pragma vector = TIMER0_B0_VECTOR
__interrupt void counter(){
    delay_counter++;
    TB0CCTL0 &= ~CCIFG;
}
