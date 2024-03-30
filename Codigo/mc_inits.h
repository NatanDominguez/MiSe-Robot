
void init_clocks(){
    
    FRCTL0 = FRCTLPW | NWAITS_1;
    P2SEL1 |= BIT6 | BIT7;

    do{
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