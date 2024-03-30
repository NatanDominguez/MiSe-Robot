
uint8_t *PTxData; // Pointer to TX data
uint8_t TXByteCtr;
uint8_t *PRxData; // Pointer to RX data
uint8_t RXByteCtr;

void i2c_init(){
	//P4SEL0 |= BIT7 + BIT6; * // P4.6 SDA i P4.7 SCL com a USCI si fem server USCI B1
	//P1SEL0 |= BIT3 + BIT2; * // P1.2 SDA i P1.3 SCL com a USCI si fem server USCI B0
	UCBxCTLW0 |= UCSWRST; // Aturem el mòdul
	//El configurem com a master, síncron i mode i2c, per defecte, està en single-master mode
	UCBxCTLW0 |= UCMST + UCMODE_3 + UCSSEL_2; // Use SMCLK,
	UCBxBR0 = 160; // fSCL = SMCLK(16MHz)/160 = ~100kHz
	UCBxBR1 = 0;
	UCBxCTLW0 &= ~UCSWRST; // Clear SW reset, resume operation
	UCBxIE |= UCTXIE0 | UCRXIE0; // Habilita les interrupcions a TX i RX
}

void I2C_receive(uint8_t addr, uint8_t *buffer, uint8_t n_dades){
	PRxData = buffer; //adreça del buffer on ficarem les dades rebudes
	RXByteCtr = n_dades; //carreguem el número de dades a rebre
	UCBxI2CSA = addr; //Coloquem l’adreça de slave
	UCBxCTLW0 &= ~UCTR; //I2C en mode Recepció
	while (UCBxCTLW0 & UCTXSTP); //Ens assegurem que el bus està en stop
	UCBxCTLW0 |= UCTXSTT; //I2C start condition en recepció
	__bis_SR_register(LPM0_bits + GIE); //Entrem en mode LPM0, enable interrupts
	__no_operation(); // Resta en mode LPM0 fins que es rebin totes les dades
}

#pragma vector = USCI_Bx_VECTOR
__interrupt void ISR_USCI_I2C(void){
	switch(__even_in_range(UCBxIV,12)){
		case USCI_NONE: break; // Vector 0: No interrupts
		case USCI_I2C_UCALIFG: break; // Vector 2: ALIFG
		case USCI_I2C_UCNACKIFG: break; // Vector 4: NACKIFG
		case USCI_I2C_UCSTTIFG: break; // Vector 6: STTIFG
		case USCI_I2C_UCSTPIFG: break; // Vector 8: STPIFG
		
		case USCI_I2C_UCRXIFG0: // Vector 10: RXIFG
		
			if (RXByteCtr){
				*PRxData++ = UCB1RXBUF; // Mou la dada rebuda a l’adreça PRxData
				if (RXByteCtr == 1) // Queda només una?
					UCBxCTLW0 |= UCTXSTP; // Genera I2C stop condition
			}

			else{
				*PRxData = UCBxRXBUF; // Mou la dada rebuda a l’adreça PRxData
				__bic_SR_register_on_exit(LPM0_bits); // Exit del mode baix consum LPM0, activa la CPU
			}

			RXByteCtr--; // Decrement RX byte counter
			break;
		
		case USCI_I2C_UCTXIFG0: // Vector 12: TXIFG
			if (TXByteCtr){ // Check TX byte counter
				UCBxTXBUF = *PTxData++; // Carrega el TX buffer amb la dada a enviar
				TXByteCtr--; // Decrementa TX byte counter
			}
			else{
				UCBxCTLW0 |= UCTXSTP; // I2C stop condition
				UCBxIFG &= ~UCTXIFG; // Clear USCI_B1 TX int flag
				__bic_SR_register_on_exit(LPM0_bits); // Exit del mode baix consum LPM0, activa la CPU
			}

		default: break;

	}
}