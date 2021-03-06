// Tilda - A two-wheel balancing robot using Lego, Attiny, Raspberry Pi
// Copyright (C) 2013 Sebastian Wangnick
// See http://s.wangnick.de/doku.php?id=tilda for design details.

#pragma once

const int MOTOR_POWER_RANGE = 127;
// const double MOTOR_VCC_SCALE = 0.01414; // Using 10k to GND and 33k to VBAT gives 700 units at 9.9V (at 3.3V VCC=AREF)
const double MOTOR_VCC_SCALE = 0.01927; // Using 10k to GND and 33k to VBAT gives 494 units at 9.52V (at 5V VCC=AREF)
const double MOTOR_RIGHT_BACKEMF_SPEED = 0.92; // V*s/360°
const double MOTOR_LEFT_BACKEMF_SPEED = 1.05; // V*s/360°
const double MOTOR_DIST_MM_SCALE = 1.0/5.8;

typedef int Sense[4];
enum {V12Y, V34Y, VCC, DIST};

int drive (int i2c, int power_right, int power_left, Sense s);

