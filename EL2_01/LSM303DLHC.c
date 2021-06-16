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
 * File:   LSM303DLHC.c
 *
 * EL2 v0.0
 *
 * LSM303DLHC accelerometer/magnetometer functions
 */

#include "LSM303DLHC.h"

//---------------------------------------------------------------------
// Initialize vector data structure
//---------------------------------------------------------------------

void InitializeVectorData(VectorData *data, bool present) {

    data->XH = 0; // x-axis high byte
    data->XL = 0; // x-axis low byte
    data->YH = 0; // y-axis high byte
    data->YL = 0; // y-axis low byte
    data->ZH = 0; // z-axis high byte
    data->ZL = 0; // z-axis low byte
    data->DRDY = 0; // data ready
    data->present = present; // sensor present?
}

//---------------------------------------------------------------------
// Configure accelerometer sensor
//---------------------------------------------------------------------

void ConfigureAccelerometer(void) {

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x20); // Set pointer to CTRL_REG1
    IdleI2C1();
    MasterWriteI2C1(0b00100111); // Set to 10 Hz, normal power mode enabled, all axes enabled
    //MasterWriteI2C1(0b00101111); // Set to 10 Hz, low power mode enabled, all axes enabled
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x21); // Set pointer to CTRL_REG2
    IdleI2C1();
    MasterWriteI2C1(0b00000000); // High pass filter bypassed
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x22); // Set pointer to CTRL_REG3
    IdleI2C1();
    MasterWriteI2C1(0b00011000); // Set INT1,INT2 set to DRDY
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x23); // Set pointer to CTRL_REG4
    IdleI2C1();
    MasterWriteI2C1(0b00000000); // +-2g full scale measurement
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Configure magnetometer sensor.
// Input argument determines whether temperature sensor is turned on.
//---------------------------------------------------------------------

void ConfigureMagnetometer(bool temp) {

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x00); // Set pointer to configuration register
    IdleI2C1();
    if (temp)
        MasterWriteI2C1(0b10000100); // Temperature sensor enabled, 1.5 Hz data ouput rate, no bias
    else
        MasterWriteI2C1(0b00000100); // Temperature sensor disabled, 1.5 Hz data ouput rate, no bias
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE);
    IdleI2C1();
    MasterWriteI2C1(0x01); // Set pointer to configuration B register
    IdleI2C1();
    MasterWriteI2C1(0b01000000); // Set gain (+- 1.9 Gauss range)
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    // Set to single conversion mode
    IdleI2C1();
    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x02); // Mode register access
    IdleI2C1();
    MasterWriteI2C1(0x01); // Single Conversion Mode, start conversion
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Start magnetometer conversion
//---------------------------------------------------------------------

void StartMagnetometer(void) {
    IdleI2C1();
    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x02); // Mode register access
    IdleI2C1();
    MasterWriteI2C1(0x01); // Single Conversion Mode, start conversion
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Put magnetometer to sleep
//---------------------------------------------------------------------

void SleepMagnetometer(void) {
    IdleI2C1();
    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x02); // Mode register access
    IdleI2C1();
    MasterWriteI2C1(0x11); // Sleep mode
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Function to read accelerometer
//---------------------------------------------------------------------

void ReadAccelerometerXYZ(VectorData *data) {

    // Read accelerometer data to buffer
    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0xA8); // First data register + msb = 1 for multiple read
    IdleI2C1();
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCREAD); // Read address
    IdleI2C1();
    data->XL = MasterReadI2C1(); // x-axis low byte
    AckI2C1();
    IdleI2C1();
    data->XH = MasterReadI2C1(); // x-axis high byte
    AckI2C1();
    IdleI2C1();
    data->YL = MasterReadI2C1(); // y-axis low byte
    AckI2C1();
    IdleI2C1();
    data->YH = MasterReadI2C1(); // y-axis high byte
    AckI2C1();
    IdleI2C1();
    data->ZL = MasterReadI2C1(); // z-axis low byte
    AckI2C1();
    IdleI2C1();
    data->ZH = MasterReadI2C1(); // z-axis high byte
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Function to read magnetometer
//---------------------------------------------------------------------

void ReadMagnetometerXYZ(VectorData *data) {

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x03); // First data register
    IdleI2C1();
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGREAD); // Read address
    IdleI2C1();
    data->XH = MasterReadI2C1(); // x-axis high byte
    AckI2C1();
    IdleI2C1();
    data->XL = MasterReadI2C1(); // x-axis low byte
    AckI2C1();
    IdleI2C1();
    data->ZH = MasterReadI2C1(); // z-axis high byte
    AckI2C1();
    IdleI2C1();
    data->ZL = MasterReadI2C1(); // z-axis low byte
    AckI2C1();
    IdleI2C1();
    data->YH = MasterReadI2C1(); // y-axis high byte
    AckI2C1();
    IdleI2C1();
    data->YL = MasterReadI2C1(); // y-axis low byte
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Function to read magnetometer temperature sensor
//---------------------------------------------------------------------

void ReadMagnetometerTemp(TempData *data) {

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x31); // First data register
    IdleI2C1();
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGREAD); // Read address
    IdleI2C1();
    data->TH = MasterReadI2C1(); // temperature high byte
    AckI2C1();
    IdleI2C1();
    data->TL = MasterReadI2C1(); // temperature low byte
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
}

//---------------------------------------------------------------------
// Function to see if the accelerometer is present
//---------------------------------------------------------------------

bool CheckAccel(void) {

    unsigned char check;

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x20); // CTRL_REG1_A
    IdleI2C1();
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(ACCREAD); // Read address
    IdleI2C1();
    check = MasterReadI2C1(); // Read byte
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    // Check that the CTRL_REG1_A has the default value (0b111)
    return (check == 0b00000111);
}


//---------------------------------------------------------------------
// Function to see if the magnetometer is present
//---------------------------------------------------------------------

bool CheckMag(void) {

    unsigned char check;

    StartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGWRITE); // Write address
    IdleI2C1();
    MasterWriteI2C1(0x0A); // IRA_REG_M register
    IdleI2C1();
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(MAGREAD); // Read address
    IdleI2C1();
    check = MasterReadI2C1(); // Read byte
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    // Check that the IRA_REG_M register has the expected value
    return (check == 0b01001000);
}
