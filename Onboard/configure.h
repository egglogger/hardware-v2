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
 * File:   configure.h
 *
 * EL2 v0.0
 *
 * PIC configuration header
 */

#include <stdio.h>
#include <stdlib.h>
#include <p24F32KA302.h>
#include <i2c.h>

#ifndef EL2_CONFIGURE_H
#define EL2_CONFIGURE_H

#define I2CBRGVAL   39	// 39 = 99 kHz, 9 = 385 kHz

/* Function prototypes */
void ConfigureTimer1(void);
void ConfigurePeripherals(void);
void ConfigureRTCC(void);
void ConfigureIO(void);
void ConfigureI2C(void);

#endif
