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
 * File:   TMP102.c
 *
 * EL2 v0.0
 *
 * TMP102 temperature sensor functions
 */
 
#include "TMP102.h"

//---------------------------------------------------------------------
// Initialize temperature data structure
//---------------------------------------------------------------------

void InitializeTempData(TempData *data, bool present) {
	data->TH = 0; // Temperature high byte
	data->TL = 0; // Temperature low byte
    data->present = present; // sensor present?
}
 
 
//---------------------------------------------------------------------
// Configure temperature sensor
//---------------------------------------------------------------------

void ConfigureTemp(unsigned char writeAddress) {
	
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0x01); // Set pointer to configuration register
	IdleI2C1();
	MasterWriteI2C1(0b01100001); // Set to power down mode
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
}

 
//---------------------------------------------------------------------
// Function to start temperature measurement
//---------------------------------------------------------------------

void StartTemp(unsigned char writeAddress) {
	
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0x01); // Set pointer to configuration register
	IdleI2C1();
	MasterWriteI2C1(0b11100001); // Start a single conversion
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
} 
 
//---------------------------------------------------------------------
// Function to read Temperature Sensor
//---------------------------------------------------------------------

void ReadTemp(TempData *data, unsigned char readAddress, unsigned char writeAddress) {
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0x00); // Set pointer to data register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress);
	IdleI2C1();
	data->TH = MasterReadI2C1(); // T high byte
	AckI2C1();
	IdleI2C1();
	data->TL = MasterReadI2C1(); // T low byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
}

//---------------------------------------------------------------------
// Function to see if the temperature sensor is present
//---------------------------------------------------------------------

bool CheckTemp(unsigned char writeAddress, unsigned char readAddress) {
	unsigned char checkHigh;
	unsigned char checkLow;
	
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0x01); // Set pointer to configuration register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress);
	IdleI2C1();
	checkHigh = MasterReadI2C1(); // high byte
	AckI2C1();
	IdleI2C1();
	checkLow = MasterReadI2C1(); // low byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
		
	// Check that the configuration register reads the default values
	return ((checkHigh == 0b01100000) && (checkLow == 0b10100000));	
}
