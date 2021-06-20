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
 * File:   TMP102.h
 *
 * EL2 v0.0
 *
 * TMP102 temperature sensor functions header
 */

#include <stdbool.h>
#include <i2c.h>
#include "EL2Data.h"
  
#ifndef TMP102_H
#define TMP102_H

/* I2C addresses in hex */
#define TEMP1READ	0x91
#define TEMP1WRITE	0x90
#define TEMP2READ	0x95
#define TEMP2WRITE	0x94
#define TEMP3READ	0x97
#define TEMP3WRITE	0x96

void InitializeTempData(TempData *data, bool present);
void ConfigureTemp(unsigned char writeAddress);
void StartTemp(unsigned char writeAddress);
void ReadTemp(TempData *data, unsigned char readAddress, unsigned char writeAddress);
bool CheckTemp(unsigned char writeAddress, unsigned char readAddress);

#endif
