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
 * File:   uSDio.c
 *
 * EL2 v0.0
 *
 * uSD card I/O functions
 */

#include <p24F32KA302.h>
#include "uSDio.h"

//---------------------------------------------------------------------
// Initialize fat file system
//---------------------------------------------------------------------

FRESULT initFS(FATFS* fs) {

    FRESULT init;

    init = f_mount(0, fs);

    /* If card initialization fails, then wait and try again */
    while (init != FR_OK) {
        // Wait 1 second
        Delay(100);
        init = f_mount(0, fs);
    }

    return init;

}

//---------------------------------------------------------------------
// Write a configuration file to test file system, report sensors
// present.
//---------------------------------------------------------------------

FRESULT writeConfigFile(int device_id) {

    FRESULT result;
    FIL file;
    char message[72];
    unsigned int bytesWritten;
    unsigned int tries;

    /* Open configuration file */
    result = f_open(&file, "config.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (result != FR_OK) {
        blinkLED(result);
        return result;
    }

    /* Write Device ID */
    bytesWritten = sprintf(message, "Device ID = %d%c%c", device_id, 0x0D, 0x0A);
    result = f_write(&file, message, strlen(message), &bytesWritten);
    f_sync(&file);
    if (result != FR_OK) {
        blinkLED(result);
        return result;
    }

    /* Close configuration file */
    tries = 0;
    result = f_close(&file);
    while (result != FR_OK && tries < FILE_CLOSE_TRIES) {
        Delay(100); /* Wait 0.1 sec */
        result = f_close(&file);
        tries++;
    }
    return result;
}

//---------------------------------------------------------------------
// Writes message to file, as specified in input arguments
//---------------------------------------------------------------------

FRESULT writeMessageToFile(const char* filename, char* message, unsigned int bytesWritten) {

    FRESULT result;
    FIL file;
    unsigned int tries;

    /* Open configuration file */
    result = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
    if (result != FR_OK) {
        blinkLED(result);
        return result;
    }

    // Append to end of file
    result = f_lseek(&file, f_size(&file));
    result = f_write(&file, message, strlen(message), &bytesWritten);
    f_sync(&file);
    if (result != FR_OK) {
        blinkLED(result);
        return result;
    }

    /* Close configuration file */
    tries = 0;
    result = f_close(&file);
    while (result != FR_OK && tries < FILE_CLOSE_TRIES) {
        Delay(100); /* Wait 0.1 sec */
        result = f_close(&file);
        tries++;
    }
    return result;
}

//---------------------------------------------------------------------
// Writes a full data buffer to a data file.  File number specified
// as function input arguments.
//---------------------------------------------------------------------

FRESULT writeDataFile(unsigned int device_id, unsigned int fileHour, BYTE *buff, unsigned int buffSize) {

    FRESULT result;
    char fileName[72];
    FIL file;
    unsigned int bytesWritten;
    unsigned int tries;

    // Open file, name based on data hour
    bytesWritten = sprintf(fileName, "%03dDAT%03d.txt", device_id, fileHour);
    result = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_WRITE);

    // Append to end of file
    result = f_lseek(&file, f_size(&file));
    result = f_write(&file, buff, buffSize, &bytesWritten);

    // Close file, make sure it closes
    tries = 0;
    result = f_close(&file);
    while (result != FR_OK && tries < FILE_CLOSE_TRIES) {
        Delay(100); /* Wait 0.1 sec */
        result = f_close(&file);
        tries++;
    }
    return result;
}
