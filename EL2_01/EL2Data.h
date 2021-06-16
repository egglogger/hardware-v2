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
 * File:   EL2Data.h
 *
 * EL2 v0.0
 *
 * Data structure definitions for use with the EL2 software
 */

#ifndef EL2DATA_H
#define EL2DATA_H

typedef unsigned char BYTE;

typedef struct {
    BYTE XL;
    BYTE XH;
    BYTE YL;
    BYTE YH;
    BYTE ZL;
    BYTE ZH;
    unsigned short int DRDY;
    bool present;
} VectorData;

typedef struct {
    BYTE TL;
    BYTE TH;
    bool present;
} TempData;

typedef struct {
    BYTE aL;
    BYTE aH;
    BYTE bL;
    BYTE bH;
    bool present;
} LightData;

typedef struct {
    VectorData accData;
    VectorData magData;
    TempData temp1Data;
    TempData temp2Data;
    TempData temp3Data;
    LightData light1Data;
    LightData light2Data;
    LightData light3Data;
} EL2Data;

#endif
