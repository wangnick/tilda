#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/time.h>
#include <math.h>

#include "mpulib.h"
#include "euler.h"

unsigned long usecs (struct timeval tv) {
	return 1000000UL*(unsigned long)tv.tv_sec+tv.tv_usec;
}

unsigned long usecs () {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return usecs(tv);
}

int main (int argc, char* argv[]) {
	int rawdata = 0;
    while (--argc) {
		argv++;
		if (0==strcmp(*argv,"-r")) {
			rawdata = 1;
		} else {
			fprintf(stderr,"Unknown option %s\n",*argv);
			return 2;
		}
	}
    int i2c = open("/dev/i2c-0", O_RDWR);
    if (i2c<0) {
		printf("Error opening I2C device: %s\n",strerror(errno));
		return 1;
	}
	Mpu mpu;
	if (mpu.init(i2c)<0) {
		return 1;
	}
	unsigned long usec = usecs();
	Euler ang;
	while (1) {
		unsigned long newusec = usecs();
		if (mpu.read(i2c)<0) {
			continue;
		}
		float acc_tilt = atan2(mpu.acc.x,mpu.acc.z)*180.0/M_PI;
		float secd = (newusec-usec)/1000000.0;
		usec = newusec;
		printf("%08lu ",usec);
		mpu.print(rawdata);
		printf(" acc_tilt[°]: %+4.1f",acc_tilt);
		ang.update(mpu,secd);
		ang.print();
		printf("\n");
	}
}
	
