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
 * File:   main.c
 *
 * EL2 v0.0
 *
 * Egg logger, main.c header
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <p24F32KA302.h>
#include <string.h>
#include <stdbool.h>
#include "configure.h"
#include "utilities.h"
#include "LSM303DLHC.h"
#include "TMP102.h"
#include "ADPS9303.h"
#include "EL2Data.h"
#include "uSDio.h"
#include "./ff9/src/integer.h"

/* Constants */
#define FOSC        8000000	// Oscillator frequency
#define FCY         FOSC/2      // Instruction cycle frequency
// bytes of data per buffer update (time[2],xAcc[2],yAcc[2],zAcc[2],
// Mag[2],yMag[2],zMag[2],tmp1[2],tmp2[2],tmp3[2],light1[4],light2[4],light3[4]
#define DATA_BYTES  32          
#define DATA_BUFFER_SIZE 16     // buffer updates per write
#define NEW_FILE_TIME 3600      // seconds per data file

#define DEVICE_ID 21 /* 0 to 999 valid */

void configureSensors(EL2Data *data);
void startSensors(EL2Data *data);
void readSensors (EL2Data *data);
