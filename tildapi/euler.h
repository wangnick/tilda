// Tilda - A two-wheel balancing robot using Lego, Attiny, Raspberry Pi
// Copyright (C) 2013 Sebastian Wangnick
// See http://s.wangnick.de/doku.php?id=tilda for design details.

#pragma once

#include "mpulib.h"

class Euler { public:
	static const float beta = 0.3f; // Amount of acceleration feedback per second
	float q0, q1, q2, q3;
	float yaw, pitch, roll;
	
	Euler ();
	void init (Mpu mpu);
	void update (Mpu mpu, float secd);
	void print ();
	
private:
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltatime);
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltatime);
};

