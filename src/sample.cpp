
/********* Hheaders *********/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

/********* Constant *********/


// Change PORT name according to your PC
// SerialPort
#define SERIAL_PORT	"/dev/ttyUSB0"


// Learn length (real_time = 0.05 * LEARNLENGTH[sec])
#define LEARNLENGTH	500


// Arm freedom (CRANE has 5 freedom)
#define ARM_FREEDOM	5


/********* END Constant *********/



/********* Type define *********/


// typedef for serial communication
typedef unsigned char uchar;


// Servomotor's data to write
typedef struct
{
	bool LED;
	bool TorqueON;

	int CW_ComplianceMargin;
	int CCW_ComplianceMargin;

	int CW_ComplianceSlope;
	int CCW_ComplianceSlope;

	double Angle;	//degree
	double Speed;	//rpm
	double Torque;	//%
}t_servoWrite;


// Servomotor's data to read
typedef struct
{
	double Angle;	//degree
	double Speed;	//rpm
	double Torque;	//%
}t_servoRead;


// Servomotor's information
typedef struct
{
	int id;
	t_servoWrite write;
	t_servoRead read;
}t_servo;


// ARMrobot's information
typedef struct
{
	t_servo servo[ARM_FREEDOM];
}t_arm;

/********* END Type define *********/



/********* Function prototype *********/


// communication functions
void openSerialPort(void);
uchar calcRobotisCheckSum(uchar *buf, int sizeofArray);
int GetStatusPacket(uchar *recv);
void CheckStatusPacket(void);
void WriteData2Bytes(uchar *buf, t_servoWrite data);
void initPacket(uchar *buf);
void RegWrite(int id, t_servoWrite data);
void Action(void);

// Time management functions
double time_diff(struct timeval pre, struct timeval post);
void wait_ms(struct timeval start, int ms);

// Initialize functions
void initServo(int id, t_servo *servo);
void initArm(t_arm *arm);

// Application functions
void getArmStatus(t_arm *arm);
void setArmStatus(t_arm *arm);
void TorqueOff(void);
void GetStatus(t_servo *servo);
void playback(void);
void copyArmData(t_arm *arm);


/********* END Function prototype *********/



/********* Global variables *********/


int fd;
t_arm armdata[LEARNLENGTH] = {0};


/********* END Global variables *********/



//open serialport
void openSerialPort(void)
{
	struct termios setting;
	
	fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
	if(fd < 0)
	{
		printf("Can't open port : %s\n",SERIAL_PORT);
		exit(-1);
	}
	
	setting.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
	
	setting.c_cc[VTIME] = 10;
	setting.c_cc[VMIN] = 10;
	setting.c_lflag = 0;
	setting.c_iflag = IGNPAR;
	setting.c_oflag = 0;	//RAW mode
	
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &setting);
	
	return;
}


//send bytes
void serialWrite(uchar *buf, int length)
{
	write(fd, buf, length);
	tcflush(fd, TCOFLUSH);
}

//calc checksum
uchar calcRobotisCheckSum(uchar *buf, int sizeofArray)
{
	int sum = 0, i;

	buf += 2;	//skip 0xff 0xff

	for(i = 2; i < (sizeofArray - 1); i++)
	{
		sum += *buf;
		buf++;
	}

	return 0xff & (~sum);
}


// get status packet
int GetStatusPacket(uchar *recv)
{
	int i;
	
	read(fd, recv, 4);
	if((recv[0] & recv[1] & 0xff) != 0xff)
	{

		printf("invalid return value:\n%02X %02X %02X %02X \n", recv[0], recv[1], recv[2], recv[3]);
		exit(-1);
	}

	read(fd, recv + 4, recv[3]);

	tcflush(fd, TCIFLUSH);

	return 4 + recv[3];
}


void CheckStatusPacket(void)
{
	uchar buf[256];
	int length;
	
	length = GetStatusPacket(buf);	
	
}


// data structure to byte array
void WriteData2Bytes(uchar *buf, t_servoWrite data)
{
	int angle = (int)( 1024.0 * (data.Angle / 300.0) );
	int speed = (int)( data.Speed / 0.111 );
	int torque = (int) ( data.Torque * 1024.0 );

	buf[0] = data.TorqueON;
	buf[1] = data.LED;
	buf[2] = data.CW_ComplianceMargin;
	buf[3] = data.CCW_ComplianceMargin;
	buf[4] = data.CW_ComplianceSlope;
	buf[5] = data.CCW_ComplianceSlope;
	buf[6] = angle & 0xff;
	buf[7] = (angle >> 8) & 0xff;
	buf[8] = speed & 0xff;
	buf[9] = (speed >> 8) & 0xff;
	buf[10] = torque & 0xff;
	buf[11] = (torque >> 8) & 0xff;

}


//initialize packet data
void initPacket(uchar *buf)
{
	buf[0] = 0xff;
	buf[1] = 0xff;
}


//register write
void RegWrite(int id, t_servoWrite data)
{
	uchar buf[19] = {0};
	
	initPacket(buf);
	buf[2] = id & 0xff;
	buf[3] = 15;
	buf[4] = 4;
	buf[5] = 24;

	WriteData2Bytes(buf + 6, data);
	buf[18] = calcRobotisCheckSum(buf, sizeof(buf));

	serialWrite(buf, sizeof(buf));
	CheckStatusPacket();
}


//get status
void GetStatus(t_servo *servo)
{
	uchar buf[8] = {0};
	uchar recv[256] = {0};
	int tmp;
	t_servoRead *r = &(servo->read);

	initPacket(buf);
	buf[2] = servo->id;
	buf[3] = 4;
	buf[4] = 2;
	buf[5] = 36;
	buf[6] = 6;
	buf[7] = calcRobotisCheckSum(buf, sizeof(buf));

	serialWrite(buf, sizeof(buf));
	
	GetStatusPacket(recv);

	r->Angle = 300.0 * (recv[5] | (recv[6] << 8)) / 1024.0;

	tmp = (recv[7] + (recv[8] << 8)) & 1023;

	if(recv[8] & 4)
	{
		r->Speed = -0.111 * tmp;
	}
	else
	{
		r->Speed = 0.111 * tmp;
	}
	
	tmp = (recv[9] + (recv[10] << 8)) & 1023;
	if(recv[10] & 4)
	{
		r->Torque = -tmp / 1024.0;
	}
	else
	{
		r->Torque = tmp / 1024.0;
	}
	

}


//send action packet
void Action(void)
{
	uchar buf[] = {0xff, 0xff, 254, 2, 5, 0};
	buf[sizeof(buf)-1] = calcRobotisCheckSum(buf, sizeof(buf));
	
	serialWrite(buf, sizeof(buf));
}


//initialize servo data
void initServo(int id, t_servo *servo)
{
	t_servoWrite *w = &(servo->write);

	servo->id = id;

	w->TorqueON = true;
	w->LED = false;
	w->CW_ComplianceMargin = 1;
	w->CCW_ComplianceMargin = 1;
	w->CW_ComplianceSlope = 32;
	w->CCW_ComplianceSlope = 32;
	w->Angle = 150;
	w->Speed = 50;
	w->Torque = 0.95;
}


//initialize arm data
void initArm(t_arm *arm)
{
	int i;
	for(i = 0; i < ARM_FREEDOM; i++)
	{
		initServo(i + 1, arm->servo + i);
	}
}


// Calc diff
double time_diff(struct timeval pre, struct timeval post)
{
	return (post.tv_usec - pre.tv_usec) + (1000000.0 * (post.tv_sec - pre.tv_sec) );
}


// Wait ms from start time
void wait_ms(struct timeval start, int ms)
{
	struct timeval now;

	do
	{
		gettimeofday(&now, NULL);
	}
	while( time_diff(start, now) < (ms * 1000.0) );

}


// Read data from arm
void getArmStatus(t_arm *arm)
{
	int i;
	for(i = 0; i < ARM_FREEDOM; i++)
	{
		GetStatus(arm->servo + i);
	}
}

// Send data to arm
void setArmStatus(t_arm *arm)
{
	int i;
	for(i = 0; i < ARM_FREEDOM; i++)
	{
		RegWrite(arm->servo[i].id, arm->servo[i].write);
	}
	Action();
}

// Learn action that occures by human hand
void learning(void)
{
	struct timeval pre;
	int i;

	TorqueOff();
	printf("torque off\n");
	for(i = 0; i < LEARNLENGTH; i++)
	{
		gettimeofday(&pre, NULL);
		getArmStatus(armdata + i);
		printf("%04d/%04d\n",i + 1, LEARNLENGTH);
		wait_ms(pre, 50);
	}
	
}

// Copy angle data
void copyArmData(t_arm *arm)
{
	int i;
	for(i = 0; i < ARM_FREEDOM; i++)
	{
		arm->servo[i].write.Angle = arm->servo[i].read.Angle;
	}
}

// Playback learned action
void playback(void)
{
	struct timeval pre;
	int i;

	printf("torque on\n");
	for(i = 0; i < LEARNLENGTH; i++)
	{
		gettimeofday(&pre, NULL);
		copyArmData(armdata + i);
		setArmStatus(armdata + i);
		printf("%04d/%04d\n",i + 1, LEARNLENGTH);
		wait_ms(pre, 50);
	}
}

// Release torque
void TorqueOff(void)
{
	int i;
	t_servo servo[5];

	for(i = 0; i < 5; i++)
	{
		initServo(i + 1, servo + i);
		servo[i].write.Torque = 0;
		RegWrite(servo[i].id, servo[i].write);
	}
	Action();
}

// Main
int main(int argc, char *argv[])
{
	int i;
	for(i = 0; i < LEARNLENGTH; i++)
	{
		initArm(armdata + i);
	}

	openSerialPort();

	TorqueOff();
	while(1)
	{
		struct timeval pre, post;
		gettimeofday(&pre, NULL);


		printf("q:quit\nl:learn\np:play\n>");
		char ch = getchar();

		switch(ch)
		{
			case 'q':
				goto exit;
			case 'l':
				printf("start learning\n");
				learning();
				break;
			case 'p':
				playback();
				break;
		}

	}

exit:
	printf("quit program\n");
	close(fd);
	
	return 0;
}






