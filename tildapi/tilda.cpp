#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>

#include "drivelib.h"
#include "mpulib.h"
#include "euler.h"

const char VERSION[] = __FILE__ ", " __DATE__" " __TIME__ ", $Id: tilda.cpp,v 1.9 2013/01/31 17:15:08 pi Exp $";

const float MOTOR_RIGHT_TORQUE = 0.060; // Nm = kg m²/s²
const float MOTOR_LEFT_TORQUE = 0.048; // Nm = kg m²/s²
const float MOTOR_TORQUE_VCC = 9.0; // V

const float MASS = 0.915; // kg
const float AXIS_DIST = 0.11; // m
const float HEIGHT = 0.22; // m
const float HEIGHT_BELOW_AXIS = 0.01; // m
const float DEPTH = 0.03; // m (vorne-hinten)

const float WHEEL_RAD = 0.041; // m - Lego 81,6 x 15
const float WHEEL_SEP = 0.07; // m

float HEIGHT_ABOVE_AXIS = HEIGHT-HEIGHT_BELOW_AXIS;
float ANGULAR_MASS = MASS/12/HEIGHT
		*(HEIGHT_ABOVE_AXIS*(DEPTH*DEPTH+4*HEIGHT_ABOVE_AXIS)
		+HEIGHT_BELOW_AXIS*(DEPTH*DEPTH+4*HEIGHT_BELOW_AXIS)); // kg m², Trägheitsmoment

float TORQUING_HEIGHT = HEIGHT-2*HEIGHT_BELOW_AXIS;
float TORQUING_MASS = MASS/HEIGHT*TORQUING_HEIGHT;
float TORQUING_DIST = TORQUING_HEIGHT/2+HEIGHT_ABOVE_AXIS;
float TORQUING_FORCE = TORQUING_MASS*Mpu::GRAV; // kg m/s², liegend
float TORQUING_TORQUE = TORQUING_FORCE*TORQUING_DIST; // kg m²/s², liegend

float AXIS_MASS = MASS-TORQUING_MASS;
float AXIS_FORCE = AXIS_MASS*Mpu::GRAV;

float UPRIGHT_PITCH = 2.4; // °
float ACC_TILT_FACTOR_PER_SECOND = 0.3;
float INERTIA_STOPTIME = 1.3; // s
float TORQUE_COUNTER_FACTOR = 1.3; // Factor without unit
float MOTOR_SENSE_FACTOR = 0.5;
float MOTOR_LOSS_FACTOR = 0.7; // Friction and other factors of generator non-efficiency
float PITCHACC_PROP_FACTOR = 0.7; // How fast to believe pitch velocity changes
float PITCHVEL_PROP_FACTOR = 0.7; // How fast to believe pitch changes
float PITCHACC_FF_TIME = 0.005; // Forward feedback
float PITCHVEL_FF_TIME = 0.01; // Forward feedback
float ELEV_CORRECTION_TIME = 1.0; // s
float POSJERK_FF_TIME = 0.005; // Forward feedback
float POSACC_FF_TIME = 0.005; // Forward feedback
float POSVEL_FF_TIME = 0.01; // Forward feedback
float POSJERK_ANG_FACTOR = 0.001; // PID
float POSACC_ANG_FACTOR = 0.003; // PID
float POSVEL_ANG_FACTOR = 0.03; // PID
float POS_ANG_FACTOR = 2.0; // PID
float POS_ANG_MAX = 2.0; // °
float YAW_POWER_FACTOR = 0.7; // Motor power units per ° of difference
float YAW_MAX_POWER_FACTOR = 10.0; // Max motor power units per ° of difference

struct Factor {
	const char* name;
	float* val;
} Factors[] = {
	#define F(x) {#x,&x}
	F(UPRIGHT_PITCH),
	F(ACC_TILT_FACTOR_PER_SECOND),
	F(INERTIA_STOPTIME),
	F(TORQUE_COUNTER_FACTOR),
	F(MOTOR_SENSE_FACTOR),
	F(MOTOR_LOSS_FACTOR),
	F(PITCHACC_PROP_FACTOR),
	F(PITCHVEL_PROP_FACTOR),
	F(PITCHACC_FF_TIME),
	F(PITCHVEL_FF_TIME),
	F(ELEV_CORRECTION_TIME),
	F(POSJERK_FF_TIME),
	F(POSACC_FF_TIME),
	F(POSVEL_FF_TIME),
	F(POSJERK_ANG_FACTOR),
	F(POSACC_ANG_FACTOR),
	F(POSVEL_ANG_FACTOR),
	F(POS_ANG_FACTOR),
	F(POS_ANG_MAX),
	F(YAW_POWER_FACTOR),
	F(YAW_MAX_POWER_FACTOR),
	{NULL,NULL}
};

void parse (char* line) {
	int len, end;
	float val;
	int res = sscanf(line,"%*s%n %f%n",&len,&val,&end);
	if (res!=1 || line[end]) {
		fprintf(stderr,"Parse error in line %s\n",line);
		return;
	}
	line[len] = 0;
	Factor* f;
	for (f = Factors; f->name; f++) {
		if (strcmp(line,f->name)==0) {
			break;
		}
	}
	if (!f->name) {
		fprintf(stderr,"Unknown factor in line %s\n",line);
		return;
	}
	*f->val = val;
	printf("%s %f\n",f->name,val);	
	fprintf(stderr,"%s %f\n",f->name,val);	
}


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
    int nooutput = 0;
    int dobalance = 0;
    // int frequency = 5;
    int rpower = 0, lpower = 0;
    while (--argc) {
		argv++;
		if (0==strcmp(*argv,"-r")) rawdata = 1;
		else if (0==strcmp(*argv,"-n")) nooutput = 1;
		else if (0==strcmp(*argv,"-b")) dobalance = 1;
		else if (0==strcmp(*argv,"-p")) {
			rpower = lpower = atoi(*++argv);
			argc--;
		//} else if (0==strcmp(*argv,"-f")) {
		//	frequency = atoi(*++argv);
		//	argc--;
		} else {
			fprintf(stderr,"Unknown option %s\n",*argv);
			return 2;
		}
	}
	
	printf("VERSION {%s}",VERSION);
	fprintf(stderr,"VERSION {%s}",VERSION);
	for (Factor* f = Factors; f->name; f++) {
		printf(" %s %f",f->name,*f->val);
		fprintf(stderr," %s %f",f->name,*f->val);
	}
	printf("\n");
	fprintf(stderr,"\n");
	
    int i2c = open("/dev/i2c-0", O_RDWR);
    if (i2c<0) {
		fprintf(stderr,"Error opening I2C device: %s\n",strerror(errno));
		return 1;
	}
	
	Mpu mpu;
	if (mpu.init(i2c)<0) {
		return 1;
	}
	Sense s;
	Mpu cal; cal.acc.x = cal.acc.y = cal.acc.z = cal.gyr.x = cal.gyr.y = cal.gyr.z = cal.mag.x = cal.mag.y = cal.mag.z = cal.temp = 0;
	int meas = 0;
	for (unsigned long unow = usecs(); usecs()-unow<200000UL; ) {
		drive(i2c,128,40,s);
		if (mpu.read(i2c)<0) continue;
		cal.acc.x += mpu.acc.x;
		cal.acc.y += mpu.acc.y;
		cal.acc.z += mpu.acc.z;
		cal.gyr.x += mpu.gyr.x;
		cal.gyr.y += mpu.gyr.y;
		cal.gyr.z += mpu.gyr.z;
		cal.mag.x += mpu.mag.x;
		cal.mag.y += mpu.mag.y;
		cal.mag.z += mpu.mag.z;
		cal.temp += mpu.temp;
		meas++;
	}
	cal.acc.x /= meas;
	cal.acc.y /= meas;
	cal.acc.z /= meas;
	cal.gyr.x /= meas;
	cal.gyr.y /= meas;
	cal.gyr.z /= meas;
	cal.mag.x /= meas;
	cal.mag.y /= meas;
	cal.mag.z /= meas;
	cal.temp /= meas;	
	float cal_acc_tilt = atan2(cal.acc.x,cal.acc.z)*180.0/M_PI;
	float cur_tilt = cal_acc_tilt;
	Euler ang;
	ang.init(cal);
	float yaw_target = ang.yaw;
	if (!nooutput) {
		printf("Calibration (%d samples): ",meas);
		cal.print(rawdata);
		ang.print();
		printf(" cal_acc_tilt[°]: %6.3f yaw_target[°]: %+4.1f\n",cal_acc_tilt,yaw_target);
	}
	
	float elev_ang = 0.0; // This can not be measured, only deduced from unexpected positional acceleration
	float pos = 0.0, pos_ang = 0.0;
	float posvel = 0.0, posvel_60ms = 0.0, posacc = 0.0, posacc_60ms = 0.0, posjerk = 0.0, posjerk_60ms = 0.0;
	float pitch = cal_acc_tilt, pitch_vel = 0.0, pitch_acc = 0.0;
	float target_pos = 0.0, target_vel = 0.0, target_acc = 0.0;
	int rmotor = s[V12Y], lmotor = s[V34Y];
	
	int fd = fileno(stdin);
	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);
	char line[80];
	char* pline = line;
	
	unsigned long ustart = usecs(), umpuold = ustart, udriveold = ustart; 
	while (1) {
		
		int c = fgetc(stdin);
		if (c=='\n') {
			*pline = 0;
			parse(line);
			pline = line;
		} else if (c>=0 && pline+1<line+sizeof(line)) {
			*pline++ = c;
		}
		
		struct timeval tv;
		gettimeofday(&tv,NULL);
		float secd = (usecs(tv)-umpuold)/1000000.0;
		if (secd<0.02) continue;
		if (mpu.read(i2c)<0) continue;
		umpuold = usecs(tv);
		mpu.acc.x += posacc_60ms/Mpu::GRAV/Mpu::ACC_SCALE;
		ang.update(mpu,secd);
		// Gerader Stand: acc.z ~ 1.0, acc.x ~ acc.y ~ 0.0, gyr ~ 0.0
		// Neigung nach vorne: acc.x nimmt zu, acc.z nimmt ab, gyr.y negativ, pitch nimmt zu
		// Neigung nach links: acc.y nimmt zu, acc.z nimmt ab, gyr.x positiv, roll nimmt zu
		// Ausrichtung Norden: yaw 0.0
		// Drehung gegen den Uhrzeigersinn (von oben gesehen): gyr.z positiv, yaw nimmt ab
		if (!nooutput) {
			char timetext[9];
			strftime(timetext,sizeof(timetext),"%T",localtime(&tv.tv_sec));
			printf("%s.%06ld ",timetext,tv.tv_usec);
			mpu.print(rawdata);
			ang.print();
		}
		
		float gyr_tilt_speed = -(mpu.gyr.y-cal.gyr.y)*Mpu::GYR_SCALE;
		float gyr_tilt_delta = gyr_tilt_speed*secd;
		float acc_tilt = atan2(mpu.acc.x,mpu.acc.z)*180/M_PI;
		float acc_tilt_factor = ACC_TILT_FACTOR_PER_SECOND*secd;
		cur_tilt = (1-acc_tilt_factor)*(cur_tilt+gyr_tilt_delta)+acc_tilt_factor*acc_tilt;
		if (!nooutput) {
			printf(" gyr_tilt_speed[°/s]: %+6.3f gyr_tilt_delta[°]: %+4.1f acc_tilt[°]: %+4.1f",
					gyr_tilt_speed, gyr_tilt_delta, acc_tilt);
		}

		float old_pitch = pitch;
		// pitch = ang.pitch;
		pitch = cur_tilt;
		float old_pitch_vel = pitch_vel;
		pitch_vel = (1-PITCHVEL_PROP_FACTOR)*pitch_vel+PITCHVEL_PROP_FACTOR*(pitch-old_pitch)/secd;
		pitch_acc = (1-PITCHACC_PROP_FACTOR)*pitch_acc+PITCHACC_PROP_FACTOR*(pitch_vel-old_pitch_vel)/secd;
		if (!nooutput) {
			printf(" pitch[°]: %+4.1f pitch_vel[°/s]: %+6.3f pitch_acc[°/s²]: %+6.3f",
					pitch, pitch_vel, pitch_acc);
		}
		
		// Feed forward
		float pitch_acc_ff = pitch_acc;
		float pitch_vel_ff = pitch_vel+pitch_acc_ff*PITCHACC_FF_TIME;
		float pitch_ff = pitch+pitch_vel_ff*PITCHVEL_FF_TIME;
		if (!nooutput) {
			printf(" pitch_ff[°]: %+4.1f pitch_vel_ff[°/s]: %+6.3f pitch_acc_ff[°/s²]: %+6.3f",
					pitch_ff, pitch_vel_ff, pitch_acc_ff);
		}
		
		float inertia_torque = ANGULAR_MASS*pitch_vel_ff/180*M_PI/INERTIA_STOPTIME;
		float torquing_ang = pitch_ff-UPRIGHT_PITCH+pos_ang;
		float torquing_torque = TORQUING_TORQUE*sin(torquing_ang/180*M_PI);
		float spine_force = TORQUING_FORCE*cos(torquing_ang/180*M_PI);
		float spine_ang = torquing_ang+elev_ang;
		float axis_force = -(spine_force*sin(spine_ang/180*M_PI)+AXIS_FORCE*sin(elev_ang/180*M_PI));
		float axis_torque = axis_force*WHEEL_RAD/2;
		float needtorque = 0;
		float yaw_power = ang.yaw-yaw_target;
		yaw_power = yaw_power<-180? yaw_power+360: yaw_power>=180? yaw_power-360: yaw_power;
		yaw_power *= YAW_POWER_FACTOR;
		yaw_power = yaw_power<-YAW_MAX_POWER_FACTOR? -YAW_MAX_POWER_FACTOR: yaw_power>YAW_MAX_POWER_FACTOR? YAW_MAX_POWER_FACTOR: yaw_power;
		if (dobalance) {
			needtorque = (inertia_torque+torquing_torque+axis_torque)*TORQUE_COUNTER_FACTOR;
			rpower = needtorque/2/(MOTOR_RIGHT_TORQUE/MOTOR_TORQUE_VCC*s[VCC]*MOTOR_VCC_SCALE)*MOTOR_POWER_RANGE+rmotor*MOTOR_LOSS_FACTOR+yaw_power;
			lpower = needtorque/2/(MOTOR_LEFT_TORQUE/MOTOR_TORQUE_VCC*s[VCC]*MOTOR_VCC_SCALE)*MOTOR_POWER_RANGE+lmotor*MOTOR_LOSS_FACTOR-yaw_power;
		}
		rpower = rpower<-MOTOR_POWER_RANGE? -MOTOR_POWER_RANGE: rpower>MOTOR_POWER_RANGE? MOTOR_POWER_RANGE: rpower;
		lpower = lpower<-MOTOR_POWER_RANGE? -MOTOR_POWER_RANGE: lpower>MOTOR_POWER_RANGE? MOTOR_POWER_RANGE: lpower;
		if (!nooutput) {
			printf(" inertia_torque[kg*m²/s²]: %+6.3f torquing_ang[°]: %+4.1f torquing_torque[kg*m²/s²]: %+6.3f",
					inertia_torque, torquing_ang, torquing_torque);
			printf(" spine_force[kg*m/s²]: %+6.4f spine_ang[°]: %+4.1f axis_force[kg*m/s²]: %+6.4f axis_torque[kg*m²/s²]: %+6.3f",
					spine_force, spine_ang, axis_force, axis_torque);
			printf(" needtorque: %+6.3f yaw_power: %+3f rpower: %+3d lpower: %+3d", needtorque, yaw_power, rpower, lpower);
		}
		int result = drive(i2c,rpower,lpower,s);
		if (result>=0) {
			secd = (usecs(tv)-udriveold)/1000000.0;
			udriveold = usecs(tv);
			rmotor = (1-MOTOR_SENSE_FACTOR)*rmotor+MOTOR_SENSE_FACTOR*s[V12Y];
			lmotor = (1-MOTOR_SENSE_FACTOR)*lmotor+MOTOR_SENSE_FACTOR*s[V34Y];
			float rvel = s[V12Y]*MOTOR_VCC_SCALE/MOTOR_RIGHT_BACKEMF_SPEED*WHEEL_RAD*2*M_PI; // m/s
			float lvel = s[V34Y]*MOTOR_VCC_SCALE/MOTOR_LEFT_BACKEMF_SPEED*WHEEL_RAD*2*M_PI; // m/s
			float posvelnew = (rvel+lvel)/2;
			float posaccnew = (posvelnew-posvel)/secd;
			posjerk = (posaccnew-posacc)/secd;
			posvel = posvelnew;
			posacc = posaccnew;
			pos += posvel*secd;
			float factor_60ms = secd/0.06;
			posvel_60ms = (1-factor_60ms)*posvel_60ms+factor_60ms*posvel;
			posacc_60ms = (1-factor_60ms)*posacc_60ms+factor_60ms*posacc;
			posjerk_60ms = (1-factor_60ms)*posjerk_60ms+factor_60ms*posjerk;
			
			// Feedforward
			float posjerk_ff = posjerk_60ms;
			float posacc_ff = posacc_60ms + posjerk_ff*POSJERK_FF_TIME;
			float posvel_ff = posvel_60ms + posacc_ff*POSACC_FF_TIME;
			float pos_ff = pos + posvel_ff*POSVEL_FF_TIME;
			
			float elev_ang_error = POSJERK_ANG_FACTOR*posjerk_ff+POSACC_ANG_FACTOR*(posacc_ff-target_acc)+POSVEL_ANG_FACTOR*(posvel_ff-target_vel);
			elev_ang = elev_ang-elev_ang_error*secd/ELEV_CORRECTION_TIME;
			
			pos_ang = POS_ANG_FACTOR*(pos_ff-target_pos); // Simple linearisation
			if (pos_ang<-POS_ANG_MAX) pos_ang = -POS_ANG_MAX; else if (pos_ang>POS_ANG_MAX) pos_ang = POS_ANG_MAX;
			
			if (!nooutput) {
				if (rawdata) {
					printf(" v12y: %3d v34y: %3d vcc: %3d dist: %3d",s[V12Y],s[V34Y],s[VCC],s[DIST]);
				} else {
					printf(" v12y[V]: %4.2f v34y[V]: %4.2f vcc[V]: %4.1f dist[cm]: %3d",
							s[V12Y]*MOTOR_VCC_SCALE, s[V34Y]*MOTOR_VCC_SCALE, s[VCC]*MOTOR_VCC_SCALE, s[DIST]);
				}
				printf(" rmotor: %+3d lmotor: %+3d", rmotor, lmotor);
				printf(" posvel[m/s]: %+6.3f posvel_60ms[m/s²]: %+6.3f pos[m]: %+5.3f target_pos[m]: %+5.3f",
						posvel, posvel_60ms, pos, target_pos);
				printf(" posacc[m/s²]: %+6.3f posacc_60ms[m/s²]: %+6.3f posjerk[m/s³]: %+6.3f posjerk_60ms[m/s³]: %+6.3f", 
						posacc, posacc_60ms, posjerk, posjerk_60ms);
				printf(" posjerk_ff[m/s³]: %+6.3f posacc_ff[m/s²]: %+6.3f posvel_ff[m/s²]: %+6.3f pos_ff[m]: %+6.3f", 
						posjerk_ff, posacc_ff, posvel_ff, pos_ff);
				printf("  elev_ang_error[°]: %+4.2f elev_ang[°]: %+4.1f  pos_ang[°]: %+4.1f",
						elev_ang_error, elev_ang, pos_ang);
			}
			
			if (usecs(tv)-ustart>60000000) {
				drive(i2c,128,20,s);
				ustart = usecs(tv);
				target_pos += 1.0;
			}
		}
		if (!nooutput) {
			printf("\n");
		}
	}
	close(i2c);
	return 0;
}
