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
 * File:   utilities.c
 *
 * EL2 v0.0
 *
 * Useful egg logger functions
 */

#include "utilities.h"

static unsigned int interruptCycles; // Static private variable for delay function

//---------------------------------------------------------------------
// Delays using Timer 1, input argument is milliseconds of delay
//---------------------------------------------------------------------

void Delay(unsigned int cycles){
    interruptCycles = 0;
    while(interruptCycles < cycles) {} /* Wait for the specificed number of milliseconds */
}

//---------------------------------------------------------------------
// Timer 1 interrupt function to use for delay()
//---------------------------------------------------------------------

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
{
    /* Clear Interrupt flag */
    IFS0bits.T1IF = 0;
    interruptCycles++;
    disk_timerproc(); /* Drive timer procedure of low level disk I/O module */
}

//---------------------------------------------------------------------
// Turn LED On
//---------------------------------------------------------------------

void LEDOn() {
    LATBbits.LATB4 = 1;
    PORTBbits.RB4 = 1;
}

//---------------------------------------------------------------------
// Turn LED Off
//---------------------------------------------------------------------

void LEDOff() {
    LATBbits.LATB4 = 0;
    PORTBbits.RB4 = 0;
}

//---------------------------------------------------------------------
// Blink LED nBlinks times
//---------------------------------------------------------------------

void blinkLED(int nBlinks) {

    int i;

    Delay(100);

    for (i = 0; i < nBlinks; i++){
        LEDOn();
        Delay(250);
        LEDOff();
        Delay(250);
    }
    
    Delay(100);
}
