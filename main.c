#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/**
 * main.c
 */

//--------------------------------------------- mc_inits

uint16_t delay_counter = 0;
uint8_t *PTxData; // Pointer to TX data
uint8_t TXByteCtr;
uint8_t *PRxData; // Pointer to RX data
uint8_t RXByteCtr;
uint8_t data_transmit[8];
uint8_t data_receive[8];
uint8_t data[3];
uint8_t dir_joystick = 0x00;
uint8_t llum_value = 0x07;

bool act, select;

uint16_t canal;
uint8_t inici_conversio = 0;

float ADC_sum = 0;
uint16_t iter = 0;
float mitjana;



//--------------------------------------------- Inicialitzacions

void init_clocks(){

    FRCTL0 = FRCTLPW | NWAITS_1;
    P2SEL1 |= BIT6 | BIT7;

    __bis_SR_register(SCG0);
    CSCTL3 |= SELREF__REFOCLK;
    CSCTL1 = DCORSEL_5;
    CSCTL2 = FLLD_0 + 487;
    __delay_cycles(3);
    __bic_SR_register(SCG0);
    CSCTL4 = SELMS__DCOCLKDIV | SELREF__REFOCLK;
    //P1DIR |= BIT0 | BIT1; // set SMCLK, ACLK pin as output
    //P1SEL1 |= BIT0 | BIT1; // set SMCLK and ACLK pin as second function
    PM5CTL0 &= ~LOCKLPM5;
}


void init_timers(){
    TB0CTL = (TBSSEL_2|MC_1);  //SMCLK i UP mode
    TB0CCTL0 = CCIE;    //capture/compare interrupt enable
    TB0CCR0 = 16000;  //1 ms
}

void delay_ms(uint8_t temps){
    /*
      Aquesta funció genera un delay a partir d'un timer
    */

    TB0CTL |= MC_1; //activem UP mode
    delay_counter = 0;
    TB0CCR0 = 16000;   // no cal però assegurem que l'escala de temps es troba en 1 ms
    while(delay_counter < temps){
        continue;
    }
    TB0CTL &= ~MC_1; //desactivem l'UP mode
}

void init_gpios(){

    P6SEL0 &= ~BIT0;
    P6SEL1 &= ~BIT0;
    P6DIR |= BIT0;
    P1OUT |= BIT0;

    //RESET LCD
    P5SEL0 &= ~BIT2;
    P5SEL1 &= ~BIT2;
    P5DIR |= BIT2;
    P5OUT = BIT2;

    //JOYSTICK
    P2SEL0 &= ~(0x0F);
    P2SEL1 &= ~(0x0F);
    P2DIR &= ~(0x0F);
    P2REN &= ~(0x0F);

    P2IE |= 0x0F;
    P2IFG &= ~(0x0F);
    P2IES |= 0x0F;

    //BOTÓ JOYSTICK
    P2SEL0 &= ~BIT4;
    P2SEL1 &= ~BIT4;
    P2DIR &= ~BIT4;
    P2REN |= BIT4;
    P2OUT &= ~BIT4;

    P2IE |= BIT4;
    P2IFG &= ~BIT4;
    P2IES |= BIT4;

}


//--------------------------------------------- Configuració de l'I2C i funcions diverses que l'empren

void i2c_init(){
    //P4SEL0 |= BIT7 + BIT6; * // P4.6 SDA i P4.7 SCL com a USCI si fem server USCI B1
    P1SEL0 |= BIT3 + BIT2; // P1.2 SDA i P1.3 SCL com a USCI si fem server USCI B0
    UCB0CTLW0 |= UCSWRST; // Aturem el mÃ²dul
    //El configurem com a master, sÃ­ncron i mode i2c, per defecte, estÃ  en single-master mode
    UCB0CTLW0 |= UCMST + UCMODE_3 + UCSSEL_2; // Use SMCLK,
    UCB0BR0 = 160; // fSCL = SMCLK(16MHz)/160 = ~100kHz
    UCB0BR1 = 0;
    UCB0CTLW0 &= ~UCSWRST; // Clear SW reset, resume operation
    UCB0IE |= UCTXIE0 | UCRXIE0; // Habilita les interrupcions a TX i RX
}

void I2C_receive(uint8_t addr, uint8_t *buffer, uint8_t n_dades){
    PRxData = buffer; //adreÃ§a del buffer on ficarem les dades rebudes
    RXByteCtr = n_dades; //carreguem el nÃºmero de dades a rebre
    UCB0I2CSA = addr; //Coloquem lâ€™adreÃ§a de slave
    UCB0CTLW0 &= ~UCTR; //I2C en mode RecepciÃ³
    while (UCB0CTLW0 & UCTXSTP); //Ens assegurem que el bus estÃ  en stop
    UCB0CTLW0 |= UCTXSTT; //I2C start condition en recepciÃ³
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
                *PRxData++ = UCB1RXBUF; // Mou la dada rebuda a lâ€™adreÃ§a PRxData
                if (RXByteCtr == 1) // Queda nomÃ©s una?
                    UCB0CTLW0 |= UCTXSTP; // Genera I2C stop condition
            }

            else{
                *PRxData = UCB0RXBUF; // Mou la dada rebuda a lâ€™adreÃ§a PRxData
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
    UCB0I2CSA = addr; //Coloquem l’adreça de slave
    PTxData = buffer; //adreça del bloc de dades a transmetre
    TXByteCtr = n_dades; //carreguem el número de dades a transmetre;
    UCB0CTLW0 |= UCTR + UCTXSTT; //I2C en mode TX, enviem la condició de start
    __bis_SR_register(LPM0_bits + GIE); //Entrem a mode LPM0, enable interrupts
    __no_operation(); //Resta en mode LPM0 fins que es trasmetin les dades
    while (UCB0CTLW0 & UCTXSTP); //Ens assegurem que s'ha enviat la condició de stop
}

void LEDS(uint8_t led_esq,uint8_t led_dret){
    data[0] = 0x0B;
    data[1] = led_esq;
    data[2] = led_dret;
    I2C_send(0x10, data, 3);
}

void motor_davant(uint8_t vel_davant, uint8_t t_ms){
    data_transmit[0] = 0x00;
    data_transmit[1] = 0x01;
    data_transmit[2] = vel_davant;
    data_transmit[3] = 0x01;
    data_transmit[4] = vel_davant;
    I2C_send(0x10, data_transmit, 5);
    delay_ms(t_ms);
}

void motor_darrere(uint8_t vel_darrere, uint8_t t_ms){
    data_transmit[0] = 0x00;
    data_transmit[1] = 0x02;
    data_transmit[2] = vel_darrere;
    data_transmit[3] = 0x02;
    data_transmit[4] = vel_darrere;
    I2C_send(0x10, data_transmit, 5);
    delay_ms(t_ms);
}

void motor_esquerra(uint8_t vel_esquerra, uint8_t t_ms){
    data_transmit[0] = 0x00;
    data_transmit[1] = 0x01;
    data_transmit[2] = vel_esquerra;
    data_transmit[3] = 0x01;
    data_transmit[4] = vel_esquerra/8;
    I2C_send(0x10, data_transmit, 5);
    delay_ms(t_ms);
}

void motor_dreta(uint8_t vel_dreta, uint8_t t_ms){
    data_transmit[0] = 0x00;
    data_transmit[1] = 0x01;
    data_transmit[2] = vel_dreta/8;
    data_transmit[3] = 0x01;
    data_transmit[4] = vel_dreta;
    I2C_send(0x10, data_transmit, 5);
    delay_ms(t_ms);
}

//--------------------------------------------- LCD

void LCD_reset(){
    P5OUT &= ~BIT2;
    delay_ms(10);
    P5OUT |= BIT2;
    delay_ms(10);
}

void LCD_init(){
    LCD_reset();
    uint8_t data_LCD_init[8];
    data_LCD_init[0] = 0x00;    //write command
    data_LCD_init[1] = 0x39;    //function set
    data_LCD_init[2] = 0x14;    //OSC frequency
    data_LCD_init[3] = 0x74;    //Contrast
    data_LCD_init[4] = 0x54;    //ICON control
    data_LCD_init[5] = 0x6F;    //Follower control
    data_LCD_init[6] = 0x0C;    //Display ON/OFF
    data_LCD_init[7] = 0x01;    //Clear

    I2C_send(0x3E, data_LCD_init, 8);

    delay_ms(100);
}

void LCD_clear(){
	char clear[2];

	clear[0] = 0x00;
	clear[1] = 0x01;

	I2C_send(0x3E, clear, 2);

	delay_ms(10);
}

//--------------------------------------------- LDRs

void LDR_init(){
    PM5CTL0 &= ~LOCKLPM5;

    P5SEL0 |= (BIT1 | BIT0);        //inicialització dels pins ADC A1 i A2
    P5SEL1 &= ~(BIT1 | BIT0);

    ADCCTL0 |= ADCSHT_2 | ADCON;    //32 clocks en el S&H i activem l'ADC

    ADCCTL1 |= ADCSSEL_0 | ADCSHP | ADCCONSEQ_0;  // Selecció de l'oscil·lador; Single Channel Single Conversion mode

    ADCCTL2 &= ~ADCRES;    
    ADCCTL2 |= ADCRES_2;            // Resolució de 12 bits, la conversió a tensió depén d'aquests 12 bits
    ADCIE = ADCIE0;                 // Habilitem interrupcions per finalització de conversions

}

float ADC_acquire(uint8_t canal) {
    ADCCTL0 &= ~ADCENC;             //Per poder modificar els registres
    ADCMCTL0 &= 0xFFF0;             //Netejem el camp del canal
    ADCMCTL0 |= canal;              //Actualitzem el canal

    delay_ms(10);     
           
    iter = 0;
    ADC_sum = 0;

    while (iter < 16) {
        ADCCTL0 |= ADCENC | ADCSC;	//habilitem conversions
        delay_ms(10);
        iter++;
    }

    return ADC_sum/16;
}

//--------------------------------------------- Funcions per realitzar les diferents funcionalitats

void control_motor_joystick(uint8_t dir_joystick, char data_LCD[18]){
    uint8_t longitud;

    if(dir_joystick == 0x00){
        longitud = sprintf(data_LCD, "@JOYSTICK: IDLE       ");
        I2C_send(0x3E, data_LCD, longitud);
        motor_davant(0x00, 10);
    }
    else if(dir_joystick == 0x01){
        longitud = sprintf(data_LCD, "@JOYSTICK: UP        ");
        I2C_send(0x3E, data_LCD, longitud);
        motor_davant(0xFF, 10);
    }
    else if(dir_joystick == 0x02){
        longitud = sprintf(data_LCD, "@JOYSTICK: DOWN      ");
        I2C_send(0x3E, data_LCD, longitud);
        motor_darrere(0xFF, 10);
    }
    else if(dir_joystick == 0x04){
        longitud = sprintf(data_LCD, "@JOYSTICK: RIGHT     ");
        I2C_send(0x3E, data_LCD, longitud);
        motor_esquerra(0xFF, 10);
    }
    else if(dir_joystick == 0x08){
        longitud = sprintf(data_LCD, "@JOYSTICK: LEFT      ");
        I2C_send(0x3E, data_LCD, longitud);

        motor_dreta(0xFF, 10);
    }
}

void control_llums(data_LCD, llums_value, llums_mode){

    uint8_t longitud;

    if(llums_mode == 0x01){  //un encès i l'altre apagat
        LEDS(0,llums_value);
        LCD_clear();
        longitud = sprintf(data_LCD, "@ LED DRET ENCÈS ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    else if(llums_mode == 0x02){  //un encès i l'altre apagat
        LEDS(llums_value,0);
        LCD_clear();
        longitud = sprintf(data_LCD, "@ LED ESQ. ENCÈS ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    else if(llums_mode == 0x03){  //dos encesos
        LEDS(llums_value,llums_value);
        LCD_clear();
        longitud = sprintf(data_LCD, "@  LEDS ENCESOS  ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    else{
    	LEDS(0,0);
    	longitud = sprintf(data_LCD, "@  LEDS APAGATS  ");
        I2C_send(0x3E, data_LCD, longitud);
    }

}

void seguir_llum(char data_LCD[18]){
    /*
     * Aquesta funció mesura la llum rebuda pels LDR i actua en conseqüència.
     *
     * Una tensió elevada als nodes dels LDR implica que hi ha llum incidint en ells, de forma que
     * en cas que un d'ells sigui superior a l'altre voldrà dir que la llum ve d'un costat.
     *
     * El rang de tensions generat pels LDRs varia entre 3.1 V i 0.1 V.
    */

    uint8_t longitud;

    float mesura_dreta, mesura_esquerra;
    float diff;

    mesura_esquerra = ADC_acquire(0x08);
    mesura_dreta = ADC_acquire(0x09);

    LCD_clear();    //nova iteració

    longitud = sprintf(data_LCD, "@ Meas. right: %x                          ", mesura_dreta);
    I2C_send(0x3E, data_LCD, longitud);
    longitud = sprintf(data_LCD, "@ Meas. left: %x                           ", mesura_esquerra);
    I2C_send(0x3E, data_LCD, longitud);

    diff = mesura_dreta - mesura_esquerra; //diferència entre nodes

    if((mesura_dreta < 1.5) && (mesura_esquerra < 1.5)){ //cas amb llum no suficient
        motor_davant(0x00, 10);
    }
    else if(diff < 0.5 && diff > -0.5){  //valor absolut de diff menor a 0.5
        motor_davant(0xFF, 10);
    }
    else{
        if(mesura_esquerra > mesura_dreta){
            motor_dreta(0xFF, 10);
        }
        else{
            motor_esquerra(0xFF, 10);
        }
    }
    delay_ms(20);
}

void seguir_linia(uint8_t data_LCD){
	/*
	* Aquesta funció preten dirigir el robot en la direcció marcada per una línia negra
	* En funció de la resposta dels sensors es redirigeix el robot en més o menys mesura.
	*/

    uint8_t longitud;
	I2C_send(0x10, 0x9D, 1);

	I2C_receive(0x9D, data_receive, 6);

	longitud = sprintf(data_LCD, "@%x", data_receive[0]);
	I2C_send(0x3E, data_LCD, longitud);

	longitud = sprintf(data_LCD, "@%x", data_receive[1]);
	I2C_send(0x3E, data_LCD, longitud);

}


//--------------------------------------------- Funció principal

int main(void){

    uint8_t longitud;

    uint8_t measure;

    uint8_t state = 0x00;
    uint8_t next = 0x00;

    uint8_t llums_value = 0x07;
    uint8_t llums_mode = 0x00;

    char data_LCD[18];

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    init_clocks();
    init_timers();
    init_gpios();

    i2c_init();
    _enable_interrupt();
    LCD_init();

    LDR_init();

    delay_ms(10);

    while(1){

    	//MENU:

    	if(state == 0x00){	//Menu principal
    	    LCD_clear();
    		//MOSTRAR INFORMACIÓ PEL LCD
    		if(next == 0x00){
    			longitud = sprintf(data_LCD, "@   ROBOT MISE                           ");
    			I2C_send(0x3E, data_LCD, 40);

    			longitud = sprintf(data_LCD, "@  NEHUEL & NATAN");
    			I2C_send(0x3E, data_LCD, longitud);

    		}
    		else if(next == 0x01){
    			longitud = sprintf(data_LCD, "@ CONTROL  LLUMS");
    			I2C_send(0x3E, data_LCD, longitud);

    		}
    		else if(next == 0x02){
    			longitud = sprintf(data_LCD, "@ CONTROL MANUAL                          ");
    			I2C_send(0x3E, data_LCD, 40);
    			longitud = sprintf(data_LCD, "@    DEL  MOTOR");
    			I2C_send(0x3E, data_LCD, longitud);
    		}
    		else if(next == 0x03){
    			longitud = sprintf(data_LCD, "@  SEGUIR  LLUM");
    			I2C_send(0x3E, data_LCD, longitud);
    		}
    		else if(next == 0x04){
    			longitud = sprintf(data_LCD, "@   LINETRACK    ");
    			I2C_send(0x3E, data_LCD, longitud);
    		}

    		//CONTROLAR L'OPCIÓ
    		if((dir_joystick == 0x08) && (act == 1)){
    			if(next < 0x04){
    				next += 1;
    			}
    		}
    		else if((dir_joystick == 0x04) && (act == 1)){
    			if(next > 0x00){
    				next -= 1;
    			}
    		}
    		act = 0;

    		if(select){	//es presiona el joystick (selecció de l'opció)
    			state = next;
    			select = 0;
    		}
    	}
    	else if(state == 0x01){	//control LEDS

    	    LCD_clear();    //nova iteració

    		if((dir_joystick == 0x08) && (llums_mode < 3) && (act)){
    			llums_mode++;
    		}
    		else if((dir_joystick == 0x04) && (llums_mode > 0) && (act)){
    			llums_mode--;
    		}
    		else if((dir_joystick == 0x02) && (llums_value < 7) && act){
    			llums_value++;
    		}
    		else if((dir_joystick == 0x01) && (llums_value > 1) && act){
    			llums_value--;
    		}

    		act = 0;
    		control_llums(data_LCD, llums_value, llums_mode);

    		if(select){
    			state = 0x00;
    			select = 0;
    		}
    	}
    	else if(state == 0x02){	//control motor joystick

    	    LCD_clear();    //nova iteració

    		control_motor_joystick(dir_joystick, data_LCD);

    		if(select){
    			state = 0x00;
    			select = 0;
    		}
    	}
    	else if(state == 0x03){ //seguiment de llum
    	    seguir_llum(data_LCD);

    	    if(select){
    	        state = 0x00;
    	        select = 0;
    	    }
    	}
    	else if(state == 0x04){	//linetrack (no funcional)
    		seguir_linia(data_LCD);

    		if(select){
    			state = 0x00;
    			select = 0;
    		}
    	}

        delay_ms(20);

    }

    return 0;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void COUNTER(){
    delay_counter++;
    TB0CCTL0 &= ~CCIFG;
}

#pragma vector = PORT2_VECTOR		//DIRECCIÓ (dir_joystick): FORWARD -> BIT0 ON; BACKWARDS -> BIT1 ON; RIGHT -> BIT2 ON; LEFT -> BIT3 ON
__interrupt void JOYSTICK_POS(){

	act = 1;

    if((P2IFG & BIT0) == BIT0){
        P2IE &= ~BIT0;
        dir_joystick ^= BIT0;

        P2IES ^= BIT0;

        P2IE |= BIT0;
        P2IFG &= ~BIT0;
    }
    if((P2IFG & BIT1) == BIT1){
        P2IE &= ~BIT1;
        dir_joystick ^= BIT1;

        P2IES ^= BIT1;

        P2IE |= BIT1;
        P2IFG &= ~BIT1;
    }
    if((P2IFG & BIT2) == BIT2){
        P2IE &= ~BIT2;
        dir_joystick ^= BIT2;

        P2IES ^= BIT2;

        P2IE |= BIT2;
        P2IFG &= ~BIT2;
    }
    if((P2IFG & BIT3) == BIT3){
        P2IE &= ~BIT3;
        dir_joystick ^= BIT3;

        P2IES ^= BIT3;

        P2IE |= BIT3;
        P2IFG &= ~BIT3;
    }
    if((P2IFG & BIT4) == BIT4){
    	P2IE &= ~BIT4;
    	select = 1;

    	P2IE |= BIT4;
    	P2IFG &= ~BIT4;
    }
}

#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void) {
    if (ADCIFG & ADCIFG0) {
        ADC_sum = ADC_sum + ADCMEM0*(3.1/4096);	//renormalització i acumulació de les mesures a ADC_sum
    }
}
