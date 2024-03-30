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

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	//init
	init_clocks();

	while(1){
	}

	return 0;
}

/*
void init_clocks(){
    FRCTL0 = FRCTLPW | NWAITS_1;
    P2SEL1 |= BIT6 | BIT7;
    do
    {
    CSCTL7 &= ~(XT1OFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
    __bis_SR_register(SCG0);
    CSCTL3 |= SELREF__XT1CLK;
    CSCTL1 = DCORSEL_5;
    CSCTL2 = FLLD_0 + 487;
    __delay_cycles(3);
    __bic_SR_register(SCG0);
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;
    P1DIR |= BIT0 | BIT1; // set SMCLK, ACLK pin as output
    P1SEL1 |= BIT0 | BIT1; // set SMCLK and ACLK pin as second function
    PM5CTL0 &= ~LOCKLPM5;
}

void init_timers(){

}

void init_gpios(){

}

void delay_ms(uint8_t temps){

    while(TA... < temps){
        continue;
    }

    delay_counter = 0;
}


#pragma TIMER_A0 {
    delay_counter++;
}
*/