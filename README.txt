Tilda - A two-wheel balancing robot using Lego, Attiny, Raspberry Pi
Copyright (C) 2014 Sebastian Wangnick (except where noted otherwise)

Tilda is a self-balancing two-wheel Lego robot with Raspberry Pi as I2C master. IMU used is an MPU-6050,
plus a HMC5883L magnetometer for absolute direction. Motors are Lego 71427, driven by an SN754410 dual H-bridge,
controlled by an Attiny861 I2C slave as PWM generator and for back-EMF and battery voltage measurement. 
Power is provided by a reused 2Ah 9.6V NiCd? rechargeable battery pack.

See http://s.wangnick.de/doku.php?id=tilda for design details.
