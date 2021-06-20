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
 * File:   utilities.h
 *
 * EL2 v0.0
 *
 * Useful egg logger functions header
 */

#include <stdio.h>
#include <stdlib.h>
#include <p24F32KA302.h>

#ifndef EL2_UTILITIES_H
#define EL2_UTILITIES_H

void Delay(unsigned int cycles);
void LEDOn();
void LEDOff();
void blinkLED(int nBlinks);
void disk_timerproc (void);

#endif
