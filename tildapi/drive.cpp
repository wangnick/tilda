#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "drivelib.h"

int main (int argc, char* argv[]) {
	char* progname = argv[0];
    int i2c, left = -MOTOR_POWER_RANGE-1, right = -MOTOR_POWER_RANGE-1, ramp = 0;
    Sense s;
    unsigned long usl = 10000;
    if (argc>1 && 0==strcmp(argv[1],"-usleep")) {
		usl = strtoul(argv[2],NULL,0);
		argc -= 2; argv += 2;
	}
	if (argc==2 && 0==strcmp(argv[1],"-ramp")) {
		ramp = 1;
		left = right = 0;
		right = 0;
	}
    if (argc==3) {
		left = atoi(argv[1]);
		right = atoi(argv[2]);
	}
	if (left<-MOTOR_POWER_RANGE || left>MOTOR_POWER_RANGE+1 || right<-MOTOR_POWER_RANGE || right>(left<=MOTOR_POWER_RANGE?MOTOR_POWER_RANGE:0xFF)) {
		fprintf(stderr,"Usage: %s [-usleep <usleep>] [-ramp] <left -127..127> <right -127..127>\n",progname);
		return 2;
	}
    i2c = open("/dev/i2c-0", O_RDWR);
    if (i2c<0) {
		fprintf(stderr,"Error opening I2C device: %s\n",strerror(errno));
		return 1;
	}
	while (1) {
		struct timeval tv;
		char timetext[9];
		gettimeofday(&tv,NULL);
		strftime(timetext,sizeof(timetext),"%T",localtime(&tv.tv_sec));
		printf("drive(i2c,%d,%d) %s.%06ld",left,right,timetext,tv.tv_usec);
		drive(i2c,left,right,s);
		for (unsigned int is=0; is<sizeof(s)/sizeof(*s); is++) {
			printf(" %+5d",s[is]);
		}
		printf("\n");
		if (left==MOTOR_POWER_RANGE+1) break;
		usleep(usl);
		if (ramp) {
			left += ramp; 
			if (left>MOTOR_POWER_RANGE) {
				ramp = -1;
				left -= 2;
			} else if (left<-MOTOR_POWER_RANGE) {
				ramp = 1;
				left += 2;
			} 
			right = left;
		}
	}
	close(i2c);
	return 0;
}
