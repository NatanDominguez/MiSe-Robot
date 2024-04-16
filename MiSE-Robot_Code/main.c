#include <msp430.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * main.c
 */

//--------------------------------------------- mc_inits

uint16_t delay_counter = 0;
uint8_t *PTxData; // Pointer to TX data
uint8_t TXByteCtr;
uint8_t *PRxData; // Pointer to RX data
uint8_t RXByteCtr;


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
    //P1DIR |= BIT0 | BIT1; // set SMCLK, ACLK pin as output
    //P1SEL1 |= BIT0 | BIT1; // set SMCLK and ACLK pin as second function
    PM5CTL0 &= ~LOCKLPM5;
}

void init_timers(){
    TB0CTL = (TBSSEL_2|MC_1);  //SMCLK i UP mode
    TB0CCTL0 = CCIE;    //capture/compare interrupt enable
    TB0CCR0 = 16000;  //1 ms

    return;
}

void init_gpios(){
}





//--------------------------------------------- i2c_ctr

void i2c_init(){
    //P4SEL0 |= BIT7 + BIT6; * // P4.6 SDA i P4.7 SCL com a USCI si fem server USCI B1
    P1SEL0 |= BIT3 + BIT2; // P1.2 SDA i P1.3 SCL com a USCI si fem server USCI B0
    UCB0CTLW0 |= UCSWRST; // Aturem el mòdul
    //El configurem com a master, síncron i mode i2c, per defecte, està en single-master mode
    UCB0CTLW0 |= UCMST + UCMODE_3 + UCSSEL_2; // Use SMCLK,
    UCB0BR0 = 160; // fSCL = SMCLK(16MHz)/160 = ~100kHz
    UCB0BR1 = 0;
    UCB0CTLW0 &= ~UCSWRST; // Clear SW reset, resume operation
    UCB0IE |= UCTXIE0 | UCRXIE0; // Habilita les interrupcions a TX i RX
}

void I2C_receive(uint8_t addr, uint8_t *buffer, uint8_t n_dades){
    PRxData = buffer; //adreça del buffer on ficarem les dades rebudes
    RXByteCtr = n_dades; //carreguem el número de dades a rebre
    UCB0I2CSA = addr; //Coloquem l’adreça de slave
    UCB0CTLW0 &= ~UCTR; //I2C en mode Recepció
    while (UCB0CTLW0 & UCTXSTP); //Ens assegurem que el bus està en stop
    UCB0CTLW0 |= UCTXSTT; //I2C start condition en recepció
    __bis_SR_register(LPM0_bits + GIE); //Entrem en mode LPM0, enable interrupts
    __no_operation(); // Resta en mode LPM0 fins que es rebin totes les dades
}

#pragma vector = USCI_B0_VECTOR
__interrupt void ISR_USCI_I2C(void){
    switch(__even_in_range(UCB0IV,12)){
        case USCI_NONE: break; // Vector 0: No interrupts
        case USCI_I2C_UCALIFG: break; // Vector 2: ALIFG
        case USCI_I2C_UCNACKIFG: break; // Vector 4: NACKIFG
        case USCI_I2C_UCSTTIFG: break; // Vector 6: STTIFG
        case USCI_I2C_UCSTPIFG: break; // Vector 8: STPIFG

        case USCI_I2C_UCRXIFG0: // Vector 10: RXIFG

            if (RXByteCtr){
                *PRxData++ = UCB1RXBUF; // Mou la dada rebuda a l’adreça PRxData
                if (RXByteCtr == 1) // Queda només una?
                    UCB0CTLW0 |= UCTXSTP; // Genera I2C stop condition
            }

            else{
                *PRxData = UCB0RXBUF; // Mou la dada rebuda a l’adreça PRxData
                __bic_SR_register_on_exit(LPM0_bits); // Exit del mode baix consum LPM0, activa la CPU
            }

            RXByteCtr--; // Decrement RX byte counter
            break;

        case USCI_I2C_UCTXIFG0: // Vector 12: TXIFG
            if (TXByteCtr){ // Check TX byte counter
                UCB0TXBUF = *PTxData++; // Carrega el TX buffer amb la dada a enviar
                TXByteCtr--; // Decrementa TX byte counter
            }
            else{
                UCB0CTLW0 |= UCTXSTP; // I2C stop condition
                UCB0IFG &= ~UCTXIFG; // Clear USCI_B1 TX int flag
                __bic_SR_register_on_exit(LPM0_bits); // Exit del mode baix consum LPM0, activa la CPU
            }

        default: break;
    }

}

void I2C_send(uint8_t addr, uint8_t *buffer, uint8_t n_dades){
    UCB0I2CSA = addr; //Coloquem l�adre�a de slave
    PTxData = buffer; //adre�a del bloc de dades a transmetre
    TXByteCtr = n_dades; //carreguem el n�mero de dades a transmetre;
    UCB0CTLW0 |= UCTR + UCTXSTT; //I2C en mode TX, enviem la condici� de start
    __bis_SR_register(LPM0_bits + GIE); //Entrem a mode LPM0, enable interrupts
    __no_operation(); //Resta en mode LPM0 fins que es trasmetin les dades
    while (UCB0CTLW0 & UCTXSTP); //Ens assegurem que s'ha enviat la condici� de stop
}


void delay_ms(uint8_t temps){
    /*
     *
     *aquesta funci� genera un delay a partir d'un timer
    */

    TB0CCR0 = 16000*temps;   // temps en ms
    while(delay_counter < temps){
        continue;
    }

    delay_counter = 0;
    return;
}


int main(void)
{
    uint8_t data_transmit[16];

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //init
    init_clocks();
    init_timers();
    init_gpios();

    i2c_init();

    _enable_interrupt();

    uint8_t texto = 0x50;
    uint8_t longitud = sprintf(data_transmit, "@veloc_esq = %d", texto);

    while(1){

        I2C_send(0x3E, data_transmit, longitud);

        delay_ms(1200);

    }

    return;
}





#pragma vector = TIMER0_B0_VECTOR
__interrupt void counter(){
    delay_counter++;
    TB0CCTL0 &= ~CCIFG;
}






