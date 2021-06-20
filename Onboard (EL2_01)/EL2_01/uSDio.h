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
 * File:   uSDio.h
 *
 * EL2 v0.0
 *
 * uSD card I/O functions header
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "utilities.h"

#include "./ff9/src/diskio.h"
#include "./ff9/src/ff.h"
#include "./ff9/src/pic24f.h"

#ifndef EL2_USD_H
#define EL2_USD_H

#define FILE_CLOSE_TRIES 9

FRESULT initFS(FATFS *fs);
FRESULT writeConfigFile(int device_id);
FRESULT writeMessageToFile(const char* filename, char* message, unsigned int bytesWritten);
FRESULT writeDataFile(unsigned int fileSecond, unsigned int fileHour, BYTE *buff, unsigned int buffSize);

#endif
