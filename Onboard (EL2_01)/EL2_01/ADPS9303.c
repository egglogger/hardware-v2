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
 * File:   ADPS9303.c
 *
 * EL2 v0.0
 *
 * ADPS9303 light sensor functions
 */
 
#include "ADPS9303.h"

//---------------------------------------------------------------------
// Initialize light data structure
//---------------------------------------------------------------------

void InitializeLightData(LightData *data, bool present) {
	data->aH = 0; // Light sensor a high byte
	data->aL = 0; // Light sensor a low byte
	data->bH = 0; // Light sensor b high byte
	data->bL = 0; // Light sensor b low byte
        data->present = present; // sensor present?
}

//---------------------------------------------------------------------
// Configure light sensor
//---------------------------------------------------------------------

void ConfigureLight(unsigned char writeAddress){
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0b10000001); // Set pointer to timing register
	IdleI2C1();
	MasterWriteI2C1(0b00010000); // Set to high gain with 13.7 ms conversion time
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
}

//---------------------------------------------------------------------
// Start light sensor
//---------------------------------------------------------------------

void StartLight(unsigned char writeAddress) {
	
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0b10000000); // Set pointer to control register
	IdleI2C1();
	MasterWriteI2C1(0x03); // Start the sensor
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
}

//---------------------------------------------------------------------
// Stop light sensor
//---------------------------------------------------------------------

void StopLight(unsigned char writeAddress) {
	
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress);
	IdleI2C1();
	MasterWriteI2C1(0b10000000); // Set pointer to control register
	IdleI2C1();
	MasterWriteI2C1(0x00); // Stop the sensor
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
}
 
//---------------------------------------------------------------------
// Read Ambient Light Sensor
//---------------------------------------------------------------------

void ReadLight(LightData *data, unsigned char readAddress, unsigned char writeAddress) {

	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress); // Write address
	IdleI2C1();
	MasterWriteI2C1(0x8C); // First data register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress); // Read address
	IdleI2C1();
	data->aL = MasterReadI2C1(); // channel 0 low byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();

	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress); // Write address
	IdleI2C1();
	MasterWriteI2C1(0x8D); // Second data register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress); // Read address
	IdleI2C1();
	data->aH = MasterReadI2C1(); // channel 0 high byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();

	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress); // Write address
	IdleI2C1();
	MasterWriteI2C1(0x8E); // First data register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress); // Read address
	IdleI2C1();
	data->bL = MasterReadI2C1(); // channel 1 low byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();

	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress); // Write address
	IdleI2C1();
	MasterWriteI2C1(0x8F); // First data register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress); // Read address
	IdleI2C1();
	data->bH = MasterReadI2C1(); // channel 1 high byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();	
}

//---------------------------------------------------------------------
// Check whether the light sensor is present
//---------------------------------------------------------------------

bool CheckLight(unsigned char writeAddress, unsigned char readAddress) {
	
	unsigned char check;
	StartI2C1();
	IdleI2C1();
	MasterWriteI2C1(writeAddress); // Write address
	IdleI2C1();
	MasterWriteI2C1(0x8A); // ID register
	IdleI2C1();
	RestartI2C1();
	IdleI2C1();
	MasterWriteI2C1(readAddress); // Read address
	IdleI2C1();
	check = MasterReadI2C1(); // ID byte
	NotAckI2C1();
	IdleI2C1();
	StopI2C1();
	IdleI2C1();
	
	// Check that the 4 most significant bits of the ID register are 0100
	return ((check & 0b11110000) == 0b01000000);
}
