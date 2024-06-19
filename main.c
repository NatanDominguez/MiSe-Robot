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
uint8_t data[3];
uint8_t dir_joystick = 0x00;
float ADC_sum = 0;
float mesura;
bool act, select;


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


//--------------------------------------------- i2c_ctr

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

void control_motor_joystick(uint8_t dir_joystick, char data_LCD[18]){
    uint8_t longitud;



    if(dir_joystick == 0x00){
        motor_davant(0x00, 1000);

        LCD_clear();
        longitud = sprintf(data_LCD, "@JOYSTICK: IDLE       ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    if(dir_joystick == 0x01){
        motor_davant(0xFF, 1000);

        LCD_clear();
        longitud = sprintf(data_LCD, "@JOYSTICK: UP        ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    if(dir_joystick == 0x02){
        motor_darrere(0xFF, 1000);

        LCD_clear();
        longitud = sprintf(data_LCD, "@JOYSTICK: DOWN      ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    if(dir_joystick == 0x04){
        motor_esquerra(0xFF, 1000);

        LCD_clear();
        longitud = sprintf(data_LCD, "@JOYSTICK: RIGHT     ");
        I2C_send(0x3E, data_LCD, longitud);
    }
    if(dir_joystick == 0x08){
        motor_dreta(0xFF, 1000);

        LCD_clear();
        longitud = sprintf(data_LCD, "@JOYSTICK: LEFT      ");
        I2C_send(0x3E, data_LCD, longitud);
    }
}

void LDR_init(){
	PM5CTL0 &= ~LOCKLPM5;
    P5SEL0 |= (BIT0 | BIT1);        //inicialització dels pins ADC A1 i A2
    P5SEL1 &= ~(BIT0 | BIT1);
    ADCCTL0 |= ADCSHT_2 | ADCON;    //Sample and hold=16 clks, ADC mode ON
    ADCCTL1 |= ADCSSEL_0 | ADCSHP;  // ADCLK=MODOSC. Font de senyal Timer intern.
    ADCCTL1 |= ADCCONSEQ_0;         //Selecció mode de funcionament = Single-channel single-conversion. 1 canal. Mode 00b
    ADCCTL2 &= ~ADCRES;             // clear ADCRES in ADCTL
    ADCCTL2 |= ADCRES_2;            // 12 bits resolution
    ADCIE = ADCIE0;                 // Habilita interrupció

    /*
	PM5CTL0 &= ~LOCKLPM5;

	P5SEL0 |= BIT0|BIT1;	//funció alternativa 
	P5SEL1 &= ~(BIT0|BIT1);

	ADCCTL0 |= (ADCSHT_2 | ADCON);	//32 cicles per mesura i habilitem l'ADC

	ADCCTL1 |= ADCSSEL_0 | ADCSHP;
	ADCCTL1 |= ADCCONSEQ_0;	//single channel single conversion

	ADCCTL2 &= ~ADCRES;
	ADCCTL2 |= ADCRES_2;	//12 bits de resolució

	ADCIE |= ADCIE0;	//habilitem les interrupcions per finalització de conversió
	*/
}

float ADC_acquire(uint8_t canal){
	/*
    

	ADCCTL0 &= ~ADCENC;

	ADCMCTL0 &= 0xFFF0;
	ADCMCTL0 |= canal;
	
	ADC_sum = 0;
	iter = 0;

	delay_ms(10);
	
	while (iter < 16){

		ADCCTL0 |= ADCENC | ADCSC;

		delay_ms(10);	//aquest delay ha de ser superior al temps de completitud d'una mesura

		iter++;
	}

	return ADC_sum/16;
	*/

	uint8_t iter;

	ADCCTL0 &= ~ADCENC;             // Deshabilitem el ADC perque si no no podem canviar el canal
    ADCMCTL0 &= 0xFFF0;             // Necessitem netejar abans d'indicar que canal volem utilitzar per no trepitjar les dades anteriors
    ADCMCTL0 |= canal;              // Selecció de canal

    delay_ms(10);                      // Fem un delay abans de fer el bucle
    ADC_sum = 0;                // Reiniciem el contador de mesures
    iter = 0;                 // Reiniciem la suma total

    // Hem de fer 16 medicions i després amb una interrupció esperar a que acabi per indicar que ha finalitzat la conversió
    while (iter < 16) {
        ADCCTL0 |= ADCENC | ADCSC;
        // El ADCENC s'utilitza per habilitar el convertidor analògic digital per realitzar conversions (Si és 1 està preparat per realitzar la conversió)
        // El ADCSC s'utilitza per iniciar una conversió ADC, quan el bit es posa a 1 el ADC comença el procés de mostreig i conversió
        // Amb el OR activem els dos registres
        delay_ms(20);

        iter++;              // Augmentem el número de mesures
    }

    return ADC_sum/16;
}

int main(void)
{
    uint8_t longitud;

    uint8_t measure;

    uint8_t state = 0x00;	//MENU: 0x00; LEDS: 0x01; MOVER ROBOT JOYSTICK: 0x02
    uint8_t next = 0x00;

    uint8_t llums_value;

    char data_LCD[18];

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //init
    init_clocks();
    init_timers();
    init_gpios();

    i2c_init();
    _enable_interrupt();
    LCD_init();

    delay_ms(10);

    while(1){

    	LCD_clear();	//nova iteració

    	//MENU:

    	if(state == 0x00){	//Menu principal
    		
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

    		//CONTROLAR L'OPCIÓ
    		if((dir_joystick == 0x08) && (act == 1)){
    			if(next < 0x03){
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

    		longitud = sprintf(data_LCD, "@MODE: %d", llums_value);
    		I2C_send(0x3E, data_LCD, longitud);

    		if(select){
    			state = 0x00;
    			select = 0;
    		}
    	}
    	else if(state == 0x02){	//control motor joystick
    		control_motor_joystick(dir_joystick, data_LCD);

    		if(select){
    			state = 0x00;
    			select = 0;
    		}
    	}

        delay_ms(10);

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
__interrupt void ADC_MEASUREMENT(){
    //només s'han habilitat les interrupcions per finalització de mesura
    if (ADCIFG & ADCIFG0) {             // Conversion ready interrupt (hi ha una interrupció)
        // El registre que guarda la dada del ADC és la ADCMEM0
        mesura = ADCMEM0;
        // Per poder normalitzar aquest número agafarem la nostra tensió màxima 3.2V
        // Com el que volem fer és fer la mitjana de les 16 mesures el que farem serà una variable que pugui emmagatzemar tot per després poder dividir
        ADC_sum = ADC_sum + mesura*(3.2/4096);


    }
}
