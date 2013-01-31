#include "drivelib.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

#define CRC_START 0xFF

unsigned char _crc_ibutton_update(unsigned char crc, unsigned char data) {
	crc = crc ^ data;
	for (unsigned char i = 0; i < 8; i++) {
	    if (crc & 0x01)
	        crc = (crc >> 1) ^ 0x8C;
	    else
	        crc >>= 1;
	}
	return crc;
}

#define I2C_MOTOR 0x26

int drive (int i2c, int power_right, int power_left, Sense s) {
	if (ioctl(i2c, I2C_SLAVE, I2C_MOTOR)<0) {
		fprintf(stderr,"ioctl error with I2C motor: %s\n",strerror(errno));
		return -1;
	}
	int r;
	unsigned char req[3], res[9], checksum;
	req[0] = power_right&0xFF;
	req[1] = power_left&0xFF;
	req[2] = _crc_ibutton_update(_crc_ibutton_update(CRC_START,req[0]),req[1]);
	errno = 0;
	if ((r = write(i2c,req,sizeof(req)))!=sizeof(req)) {
		fprintf(stderr,"Motor I2C write failed (%d): %s\n",r,strerror(errno));
		return -1;
	}
	if ((r = read(i2c,res,sizeof(res)))!=sizeof(res)) {
		fprintf(stderr,"Motor I2C read failed (%d): %s\n",r, strerror(errno));
		return -1;
	}
	checksum = CRC_START;
	for (r=0; r<(int)sizeof(res)-1; r++) checksum = _crc_ibutton_update(checksum,res[r]);
	int vcc = res[4]<<8 | res[5];
	if (vcc==0 || res[r]!=checksum) {
		fprintf(stderr,"Motor I2C comms failed: %d %d %d %d %d %d %d %d %d %d\n",res[0],res[1],res[2],res[3],res[4],res[5],res[6],res[7],res[8],checksum);
		return -1;
	}
	s[V12Y] = (signed char)res[0]<<8 | res[1];
	s[V34Y] = (signed char)res[2]<<8 | res[3];
	s[DIST] = (signed char)res[6]<<8 | res[7];
	s[VCC] = vcc;
	return vcc;
}

