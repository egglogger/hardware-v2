/*    Copyright (C) 2015  Geoffrey Bower & Alexander Naiman
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
 * File:   LSM303DLHC.h
 *
 * EL2 v0.0
 *
 * LSM303DLHC accelerometer/magnetometer functions header
 */

#include <stdbool.h>
#include <i2c.h>
#include "EL2Data.h"

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

/* I2C addresses in hex */
#define ACCREAD 	0x33
#define ACCWRITE 	0x32
#define MAGREAD		0x3D
#define MAGWRITE	0x3C

/* Function prototypes */
void InitializeVectorData(VectorData *data, bool present);
void ConfigureAccelerometer(void);
void ConfigureMagnetometer(bool temp);
void StartMagnetometer(void);
void SleepMagnetometer(void);
void ReadAccelerometerXYZ(VectorData *data);
void ReadMagnetometerXYZ(VectorData *data);
void ReadMagnetometerTemp(TempData *data);
bool CheckAccel(void);
bool CheckMag(void);

#endif
