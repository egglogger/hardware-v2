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
 * Egg logger, main software routine
 */

#include "main.h"

int main(int argc, char** argv) {

    FATFS fs; // File system object
    BYTE buff[DATA_BYTES * DATA_BUFFER_SIZE]; // Working buffer
    FRESULT result; // File system return value
    unsigned int i;
    unsigned int iBuff = 0;      // counter for data buffer size
    unsigned int iTime = 0;      // counter for data index
    unsigned int fileSecond = 0; // counter for data file size
    unsigned int fileHour = 0;   // counter for data file names

    EL2Data data; // data structure

    // Configure PIC
    ConfigureTimer1();
    ConfigurePeripherals();
    ConfigureIO();
    ConfigureI2C();

    // LED sign of life
    blinkLED(1);

    // Initialize file system
    result = initFS(&fs);

    // Write configuration file
    result = writeConfigFile(DEVICE_ID);
    // If there's a problem writing the Config File, turn the LED on
    if (result != FR_OK) {
        LEDOn();
    }

    // Configure sensors and write to config file
    configureSensors(&data);

    // Start sensors that need to be started
    startSensors(&data);

    /* Setup and enable the RTCC to wake the device up from sleep mode every second */
    ConfigureRTCC();

    while (1) {

        // Read sensors
        readSensors(&data);

        // Pack data buffer
        i = 0;
        buff[iBuff * DATA_BYTES + i++] = iTime & 0x00FF; // Low byte of time
        buff[iBuff * DATA_BYTES + i++] = (iTime & 0xFF00) >> 8; // High byte of time
        buff[iBuff * DATA_BYTES + i++] = data.accData.XL;
        buff[iBuff * DATA_BYTES + i++] = data.accData.XH;
        buff[iBuff * DATA_BYTES + i++] = data.accData.YL;
        buff[iBuff * DATA_BYTES + i++] = data.accData.YH;
        buff[iBuff * DATA_BYTES + i++] = data.accData.ZL;
        buff[iBuff * DATA_BYTES + i++] = data.accData.ZH;
        buff[iBuff * DATA_BYTES + i++] = data.magData.XL;
        buff[iBuff * DATA_BYTES + i++] = data.magData.XH;
        buff[iBuff * DATA_BYTES + i++] = data.magData.YL;
        buff[iBuff * DATA_BYTES + i++] = data.magData.YH;
        buff[iBuff * DATA_BYTES + i++] = data.magData.ZL;
        buff[iBuff * DATA_BYTES + i++] = data.magData.ZH;
        buff[iBuff * DATA_BYTES + i++] = data.temp1Data.TL;
        buff[iBuff * DATA_BYTES + i++] = data.temp1Data.TH;
        buff[iBuff * DATA_BYTES + i++] = data.temp2Data.TL;
        buff[iBuff * DATA_BYTES + i++] = data.temp2Data.TH;
        buff[iBuff * DATA_BYTES + i++] = data.temp3Data.TL;
        buff[iBuff * DATA_BYTES + i++] = data.temp3Data.TH;
        buff[iBuff * DATA_BYTES + i++] = data.light1Data.aL;
        buff[iBuff * DATA_BYTES + i++] = data.light1Data.aH;
        buff[iBuff * DATA_BYTES + i++] = data.light1Data.bL;
        buff[iBuff * DATA_BYTES + i++] = data.light1Data.bH;
        buff[iBuff * DATA_BYTES + i++] = data.light2Data.aL;
        buff[iBuff * DATA_BYTES + i++] = data.light2Data.aH;
        buff[iBuff * DATA_BYTES + i++] = data.light2Data.bL;
        buff[iBuff * DATA_BYTES + i++] = data.light2Data.bH;
        buff[iBuff * DATA_BYTES + i++] = data.light3Data.aL;
        buff[iBuff * DATA_BYTES + i++] = data.light3Data.aH;
        buff[iBuff * DATA_BYTES + i++] = data.light3Data.bL;
        buff[iBuff * DATA_BYTES + i++] = data.light3Data.bH;
        iBuff++;
        iTime++;
        fileSecond++;

        // Start a new data file at set time interval
        // (default is every hour)
        if (fileSecond == NEW_FILE_TIME) {
            fileHour++;
            fileSecond = 0;
        }

        // Write to a data file when the buffer is full
        if (iBuff >= DATA_BUFFER_SIZE) {
            // Use the LED to indicate writing so we don't unplug while writing
            LEDOn();
            result = writeDataFile(DEVICE_ID, fileHour, buff, DATA_BYTES * DATA_BUFFER_SIZE);
            iBuff = 0;
            LEDOff();
        }

        // Start the next magnetometer and temperature conversions
        startSensors(&data);

        // Turn off I2C and SPI and go to sleep for ~1 second
        I2C1CONbits.I2CEN = 0;
        SPI1STATbits.SPIEN = 0;

        Sleep(); /* Wake up thanks to RTCC alarm */

        // Turn I2C and SPI back on
        I2C1CONbits.I2CEN = 1;
        SPI1STATbits.SPIEN = 1;

    }

    return (EXIT_SUCCESS);
}

//---------------------------------------------------------------------
// Write time variable for file system
//---------------------------------------------------------------------

DWORD get_fattime(void) {
    DWORD tmr;
    BYTE rtcYear = 110, rtcMon = 10, rtcMday = 15, rtcHour = 0, rtcMin = 0, rtcSec = 0;

    _DI(); /* Critical seciton, disable interrupts */
    /* Pack date and time into a DWORD variable */
    tmr = (((DWORD) rtcYear - 80) << 25)
            | ((DWORD) rtcMon << 21)
            | ((DWORD) rtcMday << 16)
            | (WORD) (rtcHour << 11)
            | (WORD) (rtcMin << 5)
            | (WORD) (rtcSec >> 1);
    _EI(); /* Re-enable interrupts */

    return tmr;
}

//---------------------------------------------------------------------
// Clear RTCC interrupt flag
//---------------------------------------------------------------------

void __attribute__((interrupt,auto_psv)) _RTCCInterrupt(void)
{
    /* Clear Interrupt flag */
    IFS3bits.RTCIF = 0;
}

//---------------------------------------------------------------------
// Check whether sensors are present, initialize data structures, 
// and configure present sensors
//---------------------------------------------------------------------

void configureSensors(EL2Data *data) {

    FRESULT result;
    char message[72];
    const char *filename = "config.txt";
    unsigned int bytesWritten;

    bool accelPresent = false;
    bool magPresent = false;
    bool temp1Present = false;
    bool temp2Present = false;
    bool temp3Present = false;
    bool light1Present = false;
    bool light2Present = false;
    bool light3Present = false;

    // Check which sensors are present
    accelPresent = CheckAccel();
    magPresent = CheckMag();
    temp1Present = CheckTemp(TEMP1WRITE, TEMP1READ);
    temp2Present = CheckTemp(TEMP2WRITE, TEMP2READ);
    temp3Present = CheckTemp(TEMP3WRITE, TEMP3READ);
    light1Present = CheckLight(LIGHT1WRITE, LIGHT1READ);
    light2Present = CheckLight(LIGHT2WRITE, LIGHT2READ);
    light3Present = CheckLight(LIGHT3WRITE, LIGHT3READ);

    // Initialize data structures (one per sensor)
    InitializeVectorData(&(data->accData), accelPresent);
    InitializeVectorData(&(data->magData), magPresent);
    InitializeTempData(&(data->temp1Data), temp1Present);
    InitializeTempData(&(data->temp2Data), temp2Present);
    InitializeTempData(&(data->temp3Data), temp3Present);
    InitializeLightData(&(data->light1Data), light1Present);
    InitializeLightData(&(data->light2Data), light2Present);
    InitializeLightData(&(data->light3Data), light3Present);

    // Configure sensors if present
    if (accelPresent) {
        bytesWritten = sprintf(message, "LSM303DLHC accelerometer is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureAccelerometer();
    }
    if (magPresent) {
        bytesWritten = sprintf(message, "LSM303DLHC magnetometer is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        // If temp1 is NOT present, will turn on mag temp sensor
        if (!temp1Present) {
            bytesWritten = sprintf(message, "No TMP102 detected, using LSM303DLHC temperature sensor%c%c", 0x0D, 0x0A);
            result = writeMessageToFile(filename, message, bytesWritten);
        }
        ConfigureMagnetometer(!temp1Present);
    }
    if (temp1Present) {
        bytesWritten = sprintf(message, "TMP102 sensor #1 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureTemp(TEMP1WRITE);
    }
    if (temp2Present) {
        bytesWritten = sprintf(message, "TMP102 sensor #2 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureTemp(TEMP2WRITE);
    }
    if (temp3Present) {
        bytesWritten = sprintf(message, "TMP102 sensor #3 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureTemp(TEMP3WRITE);
    }

    // LIGHT SENSOR NOTE:
    // Still not sure of the best settings for the light sensors,
    // they also require 100 kHz I2C instead of 400 kHz. Not clear if it goes to
    // sleep after it's conversions time is up (if not then they draw ~0.24 mA each all the time).
    // One option is to start them up, wait the >15 ms to guarantee
    // the conversion time, read the value, then shut them off. The 15 ms can of
    // course be happening while everything else is going on. This will guarantee the PIC is
    // in a higher mode for at least 15 ms of every second. It's not clear if that's the case now.
    // Another option is to manually control the conversion time by starting the
    // light sensors first, reading everything else, stopping the light
    // sensors, then reading the light results and dividing by the time the
    // light sensors were on.
    if (light1Present) {
        bytesWritten = sprintf(message, "ADPS9303 sensor #1 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureLight(LIGHT1WRITE);
    }
    if (light2Present) {
        bytesWritten = sprintf(message, "ADPS9303 sensor #2 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureLight(LIGHT2WRITE);
    }
    if (light3Present) {
        bytesWritten = sprintf(message, "ADPS9303 sensor #3 is present%c%c", 0x0D, 0x0A);
        result = writeMessageToFile(filename, message, bytesWritten);
        ConfigureLight(LIGHT3WRITE);
    }
}

//---------------------------------------------------------------------
// Send start commands to sensors
//---------------------------------------------------------------------

void startSensors(EL2Data *data) {

    if (data->magData.present) StartMagnetometer();
    if (data->temp1Data.present) StartTemp(TEMP1WRITE);
    if (data->temp2Data.present) StartTemp(TEMP2WRITE);
    if (data->temp3Data.present) StartTemp(TEMP3WRITE);
    if (data->light1Data.present) StartLight(LIGHT1WRITE);
    if (data->light2Data.present) StartLight(LIGHT2WRITE);
    if (data->light3Data.present) StartLight(LIGHT3WRITE);

}

//---------------------------------------------------------------------
// Read data from sensors
//---------------------------------------------------------------------

void readSensors(EL2Data *data) {

    if (data->accData.present) ReadAccelerometerXYZ(&(data->accData));
    if (data->magData.present) ReadMagnetometerXYZ(&(data->magData));
    if (data->temp1Data.present) 
        ReadTemp(&(data->temp1Data), TEMP1READ, TEMP1WRITE);
    else
        ReadMagnetometerTemp(&(data->temp1Data));
    if (data->temp2Data.present) ReadTemp(&(data->temp2Data), TEMP2READ, TEMP2WRITE);
    if (data->temp3Data.present) ReadTemp(&(data->temp3Data), TEMP3READ, TEMP3WRITE);
    if (data->light1Data.present) ReadLight(&(data->light1Data), LIGHT1READ, LIGHT1WRITE);
    if (data->light2Data.present) ReadLight(&(data->light2Data), LIGHT2READ, LIGHT2WRITE);
    if (data->light3Data.present) ReadLight(&(data->light3Data), LIGHT3READ, LIGHT3WRITE);

}
