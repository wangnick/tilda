#include "mpulib.h"

#include <stdio.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define I2C_MPU6050 0x69
#define MPU6050_POWER 0x6B
#define MPU6050_POWER_SLEEP 0x40
#define MPU6050_POWER_CLKSEL 0x07
#define MPU6050_POWER_CLKSEL_GYROX 0x01
//#define MPU6050_RESET 0x68
//#define MPU6050_RESET_ALL 0x07
#define MPU6050_USERCTRL 0x6A
#define MPU6050_USERCTRL_RESET 0x01
#define MPU6050_DATA 0x3B

#define I2C_HMC5883L 0x1E
#define HMC5883L_CONFIGA 0x00
#define HMC5883L_CONFIGA_INIT 0x78 // 8 samples averaged, 75Hz output rate, normal measurement
#define HMC5883L_CONFIGB 0x01
#define HMC5883L_CONFIGB_INIT 0x40 // -2048-2047 +-1.9 Gauss range, 820 LSB per Gauss
#define HMC5883L_MODE 0x02
#define HMC5883L_MODE_INIT 0x00 // Continuous measurement mode
#define HMC5883L_DATA 0x03
#define HMC5883L_STATUS 0x09
#define HMC5883L_IDENT 0x0A
#define HMC5883L_IDENT_STRING "H43"

const int ACAL = -40;
const int BCAL = -23;
const int CCAL = -272;

inline static int16_t convert (__u8 hi, __u8 lo) {
	return hi<<8 | lo;
}

int Mpu::init (int i2c) {
    int res;
	if (ioctl(i2c, I2C_SLAVE, I2C_MPU6050)<0) {
		fprintf(stderr,"ioctl error with I2C_MPU6050 device: %s\n",strerror(errno));
		return -1;
	}
	if ((res = i2c_smbus_read_byte_data(i2c,MPU6050_USERCTRL))<0) {
		fprintf(stderr,"Reading I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	res |= MPU6050_USERCTRL_RESET;
	if (i2c_smbus_write_byte_data(i2c,MPU6050_USERCTRL,res)<0) {
		fprintf(stderr,"Writing I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	if ((res = i2c_smbus_read_byte_data(i2c,MPU6050_POWER))<0) {
		fprintf(stderr,"Reading I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	res &= ~MPU6050_POWER_SLEEP;
	res &= MPU6050_POWER_CLKSEL;
	res |= MPU6050_POWER_CLKSEL_GYROX;
	if (i2c_smbus_write_byte_data(i2c,MPU6050_POWER,res)<0) {
		fprintf(stderr,"Writing I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	if (ioctl(i2c, I2C_SLAVE, I2C_HMC5883L)<0) {
		fprintf(stderr,"ioctl error with I2C_HMC5883L device: %s\n",strerror(errno));
		return -1;
	}
	__u8 data[4];
	data[sizeof(data)-1] = 0;
	res = i2c_smbus_read_i2c_block_data(i2c,HMC5883L_IDENT,sizeof(data)-1,data);
	if (res != sizeof(data)-1) {
		fprintf(stderr,"Reading I2C_HMC5883L data block failed: %s\n",strerror(errno));
		return -1;
	}
	if (strcmp((char*)data,HMC5883L_IDENT_STRING)) {
		fprintf(stderr,"I2C_HMC5883L ident is %s, not %s as expected\n",data,HMC5883L_IDENT_STRING);
		return -1;
	}

	if (i2c_smbus_write_byte_data(i2c,HMC5883L_CONFIGA,HMC5883L_CONFIGA_INIT)<0) {
		fprintf(stderr,"Writing I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	if (i2c_smbus_write_byte_data(i2c,HMC5883L_CONFIGB,HMC5883L_CONFIGB_INIT)<0) {
		fprintf(stderr,"Writing I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	if (i2c_smbus_write_byte_data(i2c,HMC5883L_MODE,HMC5883L_MODE_INIT)<0) {
		fprintf(stderr,"Writing I2C_MPU6050 byte failed: %s\n",strerror(errno));
		return -1;
	}
	credtemp = 0;
	usleep(100000); // Let MPU6050 initialize
	return 0;
}	

int Mpu::read (int i2c) {
	__u8 data[14];
	if (ioctl(i2c, I2C_SLAVE, I2C_MPU6050)<0) {
		fprintf(stderr,"ioctl error with I2C_MPU6050 device: %s\n",strerror(errno));
		return -1;
	}
	int res = i2c_smbus_read_i2c_block_data(i2c,MPU6050_DATA,sizeof(data),data);
	if (res != sizeof(data)) {
		fprintf(stderr,"Reading I2C_MPU6050 data block failed: %s\n",strerror(errno));
		return -1;
	}
	do {
		if (data[res-1]) break;
	} while (--res);
	if (res==0) {
		fprintf(stderr,"Ignoring empty I2C_MPU6050 data block\n");
		return -1;
	}
	int16_t newtemp = convert(data[6],data[7]);
	if (credtemp && fabs((temp-newtemp)*TEMP_SCALE)>2.0) {
		if (credtemp==1) credtemp = 0;
		fprintf(stderr,"Ignoring I2C_MPU6050 data block with incredible temp %+5d (last temp %+5d)\n",newtemp,temp);
		credtemp = 0;
		return -1;
	}
	if (credtemp<2) credtemp++;
	__u8 data2[6];
	if (ioctl(i2c, I2C_SLAVE, I2C_HMC5883L)<0) {
		fprintf(stderr,"ioctl error with I2C_HMC5883L device: %s\n",strerror(errno));
		return -1;
	}
	res = i2c_smbus_read_i2c_block_data(i2c,HMC5883L_DATA,sizeof(data2),data2);
	if (res != sizeof(data2)) {
		fprintf(stderr,"Reading I2C_HMC5883L data block failed: %s\n",strerror(errno));
		return -1;
	}
	temp = newtemp;
	acc.x = convert(data[0],data[1]);
	acc.y = convert(data[2],data[3]);
	acc.z = convert(data[4],data[5]);
	gyr.x = convert(data[8],data[9]);
	gyr.y = convert(data[10],data[11]);
	gyr.z = convert(data[12],data[13]);
	mag.y = convert(data2[0],data2[1])-ACAL;
	mag.z = BCAL-convert(data2[2],data2[3]);
	mag.x = CCAL-convert(data2[4],data2[5]);
	return 0;
}

void Mpu::print (int rawdata) {
	if (rawdata) {
		printf(" acc: %+5d %+5d %+5d",acc.x, acc.y, acc.z);
		printf(" gyro: %+5d %+5d %+5d",gyr.x, gyr.y, gyr.z);
		printf(" mag: %+5d %+5d %+5d",mag.x, mag.y, mag.z);
		printf(" temp: %+5d",temp);
	} else {
		printf(" acc[m/s²]: %+7.3f %+7.3f %+7.3f",acc.x*ACC_SCALE*GRAV, acc.y*ACC_SCALE*GRAV, acc.z*ACC_SCALE*GRAV);
		printf(" gyro[°/s]: %+8.3f %+8.3f %+8.3f",gyr.x*GYR_SCALE, gyr.y*GYR_SCALE, gyr.z*GYR_SCALE);
		printf(" mag[uT]: %+5.1f %+5.1f %+5.1f",mag.x*MAG_SCALE, mag.y*MAG_SCALE, mag.z*MAG_SCALE);
		printf(" temp[°C]: %+4.1f",temp*TEMP_SCALE+TEMP_OFF);
	}
}


