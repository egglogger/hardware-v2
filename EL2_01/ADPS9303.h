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
 * File:   ADPS9303.h
 *
 * EL2 v0.0
 *
 * ADPS9303 light sensor functions header
 */

#include <stdbool.h>
#include <i2c.h>
#include "EL2Data.h"
  
#ifndef ADPS9303_H
#define ADPS9303_H

/* I2C addresses in hex */
#define LIGHT1READ	0x53
#define LIGHT1WRITE	0x52
#define LIGHT2READ	0x73
#define LIGHT2WRITE	0x72
#define LIGHT3READ	0x93
#define LIGHT3WRITE	0x92

void InitializeLightData(LightData *data, bool present);
void ConfigureLight(unsigned char writeAddress);
void StartLight(unsigned char writeAddress);
void StopLight(unsigned char writeAddress);
void ReadLight(LightData *data, unsigned char readAddress, unsigned char writeAddress);
bool CheckLight(unsigned char writeAddress, unsigned char readAddress);

#endif
