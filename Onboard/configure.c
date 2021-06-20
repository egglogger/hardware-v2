/*    Copyright (C) 2015  Geoffrey Bower & Alex Naiman
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * File:   configure.c
 *
 * EL2 v0.0
 *
 * PIC configuration functions
 */

#include "configure.h"
#include "main.h"

/* Device configuration Registers */
/* Select Internal FRC at POR, set SOSCI pin to digital */
_FOSCSEL(FNOSC_FRC & SOSCSRC_DIG );
/* Enable Clock Switching, disable Failsafe clock monitor, and CLK0 output disabled */
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);
/* Disable watchdog timer, but set prescaler and postscalers for software implementation */
/* Sets software watchdog to one second sleep time period */
_FWDT(FWDTEN_SWON & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS256);

//---------------------------------------------------------------------
// Configure peripherals to reduce power consumption
//---------------------------------------------------------------------

void ConfigurePeripherals(void) {
	PMD1bits.ADC1MD = 1; // Disable ADC
	PMD1bits.SPI2MD = 1; // Disable SPI2
	PMD1bits.U1MD = 1; // Disable UART1 
	PMD1bits.U2MD = 1; // Disable UART2	
	PMD1bits.T2MD = 1; // Disable Timer 2
	PMD1bits.T3MD = 1; // Disable Timer 3
	PMD1bits.T4MD = 1; // Disable Timer 4
	PMD1bits.T5MD = 1; // Disable Timer 5
	PMD2 = 0xFF; // Disable all input capture and output compare peripherals
	PMD3bits.CMPMD = 1; // Disable CMP
	PMD3bits.CRCPMD = 1; // Disable CRC
	PMD3bits.I2C2MD = 1; // Disable I2C2
	PMD4 = 0xFF; // Disable power related peripherals HLVD, ULPWU, EE, REF, CTMU
}

//---------------------------------------------------------------------
// Configure real time clock and calendar (RTCC) for 1 Hz interrupt
//---------------------------------------------------------------------

void ConfigureRTCC(void) {

    __builtin_write_RTCWEN(); /* Built in RTCC write enable function */
	//RCFGCALbits.RTCWREN = 1; /* Enable writing to RTCVAL registers */
	RTCPWCbits.RTCCLK = 0b01; /* Select LPRC as the clock source */
	
	/* Set the time to Jan 1, 2000 @ 00:00:00
	 * Note these are BCD values, each 4 bits represents the 
	 * binary value of each decimal value.
	 * Also, the pointer to the RTCVAL is automatically 
	 * incremented after write instructions */
	RCFGCALbits.RTCPTR = 3;
	RTCVAL = 0x2000; /* Year */
	RTCVAL = 0x0101; /* Month / Day */
	RTCVAL = 0x0000; /* Day of week / Hour */
	RTCVAL = 0x0000; /* Min / Sec */
	
	ALCFGRPTbits.ALRMPTR = 2; 
	ALRMVAL = 0x0101; /* Month / Day */
	ALRMVAL = 0x0000; /* Day of week / Hour */
	ALRMVAL = 0x0001; /* Min / Sec */
	
	/* Set alarm for every second */
	ALCFGRPTbits.CHIME = 1; /* Chime in enabled, keep triggering the alarm */
	ALCFGRPTbits.AMASK = 0b0001; /* Set alarm every second */
	ALCFGRPTbits.ARPT = 255; /* Set alarm to repeat 255 times (will repeat) */
	ALCFGRPTbits.ALRMEN = 1; /* Enable alarm */		
	
	IEC3bits.RTCIE = 1; /* Enable RTCC interrupt */
	
	RCFGCALbits.RTCEN = 1; /* Turn on RTCC */
	RCFGCALbits.RTCWREN = 0; /* Lock the RTCC config registers */
}

//---------------------------------------------------------------------
// Configure timer 1 at 1000 Hz to use for delay function
//---------------------------------------------------------------------

void ConfigureTimer1(void) {

    ////////////// Configure Timer1 ///////////////
    IFS0bits.T1IF = 0;          // Clears Timer1 interrupt status flag
    IEC0bits.T1IE = 0;          // Disables Timer1 interrupts
    T1CONbits.TON = 0; 			// Timer off
    T1CONbits.TSIDL = 0;        // Continue in idle mode
    T1CONbits.TCKPS = 0b00;     // Prescaler of 1
    T1CONbits.TCS = 0; 			// Internal clock
    T1CONbits.TGATE = 0;        // Disable gated timer
    TMR1 = 0x0000; 				// Clear timer
    PR1 = FCY/1000;				// Set timer period (4000 = 1 ms)
    T1CONbits.TON = 1; 			// Timer on
    _T1IE = 1;                  // Enable timer 1 interrupt
}

//---------------------------------------------------------------------
// Configure all I/O pins
//---------------------------------------------------------------------

void ConfigureIO(void) {

    /* Set all analog pins to digital*/
    ANSA = 0x0000;
    ANSB = 0x0000;

    /* Define Inputs */
	/* Pin 9 - Magnetometer DRDY, RA9 */
	TRISAbits.TRISA4 = 1;

    /* Pin 11 - ACCINT1, RB5 */
	TRISBbits.TRISB5 = 1;

	/* Pin 12 - ACCINT2, RB6 */
	TRISBbits.TRISB6 = 1;

    /* Pin 18 - MISO, RB10 (may not be needed) */
    TRISBbits.TRISB10 = 1;


	/* Define Outputs */
	/* Pin 8 - LED , RB4 */
	TRISBbits.TRISB4 = 0;
	LATBbits.LATB4 = 0;
	PORTBbits.RB4 = 0;

    /* Pin 21 - MOSI, RB13 (may not be needed) */
    TRISBbits.TRISB13 = 0;

    /* Pin 22 - CS, RB14 */
    TRISBbits.TRISB14 = 0;
    LATBbits.LATB14 = 1; /* High, so don't select SD card initially (selected when low) */
}

//---------------------------------------------------------------------
// Configure I2C Port and I2C peripherals
//---------------------------------------------------------------------

void ConfigureI2C(void) {

	// Configure I2C Port to communicate with the magnetometer, and accelerometer
	// Disable I2C Port
	I2C1CONbits.I2CEN = 0;

	// Continue operation in idle mode
	I2C1CONbits.I2CSIDL = 0;

	// Clock stretch enable (when acting as slave)
	I2C1CONbits.SCLREL = 0;

	// Intelligent peripheral management bit
	I2C1CONbits.IPMIEN = 0;

	// Enable 7 bit addressing
	I2C1CONbits.A10M = 0;

	// Disable slew rate control bit
	I2C1CONbits.DISSLW = 1;

	// SMBus input levels bit
	I2C1CONbits.SMEN = 0;

	// General call enable bit (when acting as slave)
	I2C1CONbits.GCEN = 0;

	// Clock stretch enable bit (when acting as slave)
	I2C1CONbits.STREN = 0;

	// Acknowledge data bit (1 = NACK, 0 = ACK)
	I2C1CONbits.ACKDT = 1;

	// Acknowledge enable bit - sends acknowledgment
	I2C1CONbits.ACKEN = 0;

	// Receive enable bit
	I2C1CONbits.RCEN = 0;

	// Stop enable bit
	I2C1CONbits.PEN = 0;

	// Repeated start enable bit
	I2C1CONbits.RSEN = 0;

	// Start enable bit
	I2C1CONbits.SEN = 0;

	// Set the I2C Baud rate to 400 kHz
	I2C1BRG = I2CBRGVAL;

	// Enable I2C Port
	I2C1CONbits.I2CEN = 1;
}
