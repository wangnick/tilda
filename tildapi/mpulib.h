#pragma once

#include <fcntl.h>

class Mpu3d { public:
	int x, y, z;
};

class Mpu { public:
	// Gerader Stand: acc.z ~ 1.0, acc.x ~ acc.y ~ 0.0, gyr ~ 0.0
	// Neigung nach vorne: acc.x nimmt zu, acc.z nimmt ab, gyr.y negativ
	// Neigung nach links: acc.y nimmt zu, acc.z nimmt ab, gyr.x positiv
	// Drehung gegen den Uhrzeigersinn (von oben gesehen): gyr.z positiv
	Mpu3d acc;
	int temp;
	Mpu3d gyr;
	Mpu3d mag;
	int init (int i2c);
	int read (int i2c);
	void print (int rawdata);
	static const float GRAV = 9.81; // m/s²
	static const float ACC_SCALE = 2.0/0x8000; // g
	static const float GYR_SCALE = 250.0/0x8000; // °/s
	static const float TEMP_SCALE = 1.0/340; // °C
	static const float TEMP_OFF = 36.53; // °C
	static const float MAG_SCALE = 100.0/820; // uT
private:
	unsigned credtemp;
};



