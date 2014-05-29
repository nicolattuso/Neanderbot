#include <stdio.h>  
#include <stdlib.h>  
#include <unistd.h>  
#include <signal.h>  
#include <string.h>  
#include <errno.h>  
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

//=========================================
/// Constants
//=========================================
#define PI 3.141592654
// Coords table
#define COORD_X_MAX 30000 //mm
#define COORD_Y_MAX 20000 //mm


#define DEG2RAD PI/180;
#define RAD2DEG 180/PI;

//----------------------------------------------------------------
// Pin definition
// Sonar
#define TRIGGER_PIN 0  
#define ECHO_PIN  7  
#define SENSOR_DEBUG_PIN 3
#define TIMEOUT 999 
// Jack
#define JACK_PIN 1
// Color Select
#define COLOR_SELECT 2
//relay control
//?

#define DIST_THRESHOLD 10
//cm


typedef enum RobState 
{ 
	LOAD, WAIT, INIT, CHECK_START, STRAT, ACTION, BLOCKED, RECUP, END 
}RobState;

typedef enum ActionType {
	ACTION_MOVE,
	ACTION_WAIT,
	ACTION_TURN,
	ACTION_STOP,
	NUM_ACTIONS
} ActionType;

//----------------------------------------------------------------
// Global vars
 int sensor_dist;  //pilock0
 int sonar_enabled;//pilock1
 static volatile RobState state;
 
 int started;
 //---- Asserv ---------------------------------------------------
 static volatile int finished;   //pilock2
 int cmd_val; //pilock3

/// Parametrage
    long FOOTING;	    //distance entre les centres des roues codeuses
      ///rayon roues (mm)
    long WHEEL_RADIUS;
      ///resolution encodeurs (ticks/tr)
    long ENCODERS_RES;

    long ROBOT_RADIUS; // size of robot boundaries

    static double MMPARTICK;     //conversion ticks -> mm
    static double TICKPARMM;     //              mm -> ticks
unsigned int round_start_time;

//================================================================
//   MD25
//================================================================
#define Speed1       0x00
#define Speed2       0x01
#define Enc1a        0x02
#define Enc1b        0x03
#define Enc1c        0x04
#define Enc1d        0x05
#define Enc2a        0x06
#define Enc2b        0x07
#define Enc2c        0x08
#define Enc2d        0x09
#define Battery      0x0A
#define M1Current    0x0B
#define M2Current    0x0C
#define SWver        0x0D
#define Acceleration 0x0E
#define Mode         0X0F
#define Command      0X10
/***************************************************************
// COMMAND REG.s
//**************************************************************/
#define Reset_encoder        0x20
#define Disable_controller   0x30
#define Enable_controller    0x31
#define Disable_timeout      0x32
#define Enable_timeout       0x33
#define Change_i2c_address_1 0xA0
#define Change_i2c_address_2 0xAA
#define Change_i2c_address_3 0xA5

int fd;	                      //File description
char *fileName = "/dev/i2c-1";// Name of the port we will be using
int  address = 0x58;          // Address of MD25 shifted one bit
unsigned char buf[10];        // Buffer for data being read/ written on the i2c bus
double m_c1,m_c2;

double mmToStep (double mm)
{
	return mm * TICKPARMM;
}
// Convert steps to mm.
double stepToMm (double steps)
{
	return steps * MMPARTICK;
}
double degToRad (double a_deg)
{
	return a_deg * DEG2RAD;
}
// Convert rad to deg.
double radToDeg (double a_rad)
{
	return a_rad * RAD2DEG;
}
void resetEncoders(void)
{
	buf[0] = 16; // Command register
	buf[1] = 32; // command to set decoders back to zero
	if ((write(fd, buf, 2)) != 2)
	{
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}
long readEncoderValues (void)
{
	buf[0] = 2; // register for start of encoder values

	if ((write(fd, buf, 1)) != 1) { printf("Error writing to i2c slave\n"); exit(1); }
    if (read(fd, buf, 8) != 8) // Read back 8 bytes for the encoder values into buf[]
    {
        fprintf(stderr, "MD25::ERROR >> readEncoderValues(): Unable to read from slave\n");
        return -1;
    }
	else
	{ 
        m_c1 = (buf[0] <<24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]; // Put encoder values together
        m_c2 = (buf[4] <<24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
        //printf("Encoder 1: %08lX Encoder 2: %08lX\n",encoder1, encoder2);
	}
    return m_c1;
}
void driveMotors(void)
{
	buf[0] = Speed1; // Register to set speed of motor 1
	buf[1] = 180;//200; // speed to be set
	if ((write(fd, buf, 2)) != 2) 
	{ 
		printf("Error writing to i2c slave\n"); 
		exit(1); 
	}
	buf[0] = Speed2; // motor 2 speed
	buf[1] = 180;//200; 
	if ((write(fd, buf, 2)) != 2) 
	{ 
		printf("Error writing to i2c slave\n");
		exit(1); 
	}
}
void turnMotors(int angle)
{
	int clockwise = (angle > 0);
	
    buf[0] = Speed1; // Register to set speed of motor 1
    buf[1] = (clockwise)?(55):(200); // speed to be set (reverse if needed)
    if ((write(fd, buf, 2)) != 2)
    {
        printf("Error writing to i2c slave\n");
        exit(1);
    }
    buf[0] = Speed2; // motor 2 speed
    buf[1] = (clockwise)?(200):(55);;
    if ((write(fd, buf, 2)) != 2)
    {
        printf("Error writing to i2c slave\n");
        exit(1);
    }
}

void stopMotors(void)
{ 
	buf[0] = Speed1;
	buf[1] = 128; // A speed of 128 stops the motor
	if ((write(fd, buf, 2)) != 2)
	{
		printf("Error writing to i2c slave\n"); 
		exit(1);
	}
	buf[0] = Speed2;
	buf[1] = 128;
	if ((write(fd, buf, 2)) != 2) 
	{
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

PI_THREAD (move)
{
	(void)piHiPri (10) ;
		printf ("Move Enter.(%d mm)", cmd_val);
	piLock(2);
    finished = 0;
	piUnlock(2);
	
	
	piLock(3);
	double ticks = mmToStep(cmd_val);
	piUnlock(3);
	
	double sonarEcho;
	
    while(readEncoderValues() < ticks)
    { // Check the value of encoder 1 and stop after it has traveled a set distance
        piLock(0);
		sonarEcho = sensor_dist/58;
		piUnlock(0);
		
		if(sonar_enabled && sonarEcho < DIST_THRESHOLD)
		{
			puts("Obstacle !!!");
			stopMotors();
		}
		else
			driveMotors();
    }
    stopMotors();
	
	piLock(2);
    finished = 1;
	piUnlock(2);
}

PI_THREAD (turn)
{
	printf ("turn Enter.(%d mm)", cmd_val);
	piLock(2);
    finished = 0;
	piUnlock(2);
	resetEncoders();
	
	(void)piHiPri (10) ;
	
	piLock(3);
	double consign = mmToStep(ROBOT_RADIUS * degToRad(cmd_val));
	piUnlock(3);
	
    //set motors to opposite speeds while traveled arc < consign
    while(readEncoderValues() < consign)
    { // Check the value of encoder 1 and stop after it has traveled a set distance
        turnMotors(consign);
    }
	stopMotors();
	
	piLock(2);
    finished = 1;
	piUnlock(2);
}
//================================================================
//   SONAR MODULE
//================================================================
int waitforpin(int pin, int level, int timeout)  
{  
	struct timeval now, start;  
	int done;  
	long micros;  
	gettimeofday(&start, NULL);  
	micros = 0;  
	done=0;  
	while (!done)  
	{  
		gettimeofday(&now, NULL);  
		if (now.tv_sec > start.tv_sec) micros = 1000000L; else micros = 0;  
		micros = micros + (now.tv_usec - start.tv_usec);  
		if (micros > timeout) done=1;  
		if (digitalRead(pin) == level) done = 1;  
	}  
	return micros;  
}  

PI_THREAD (sonarReading)
{
	int state = 0 ;
	int debounceTime = 0 ;
	int pulsewidth; 

	(void)piHiPri (10) ;	// Set this thread to be high priority

	puts ("....sonar online.");
	for (;;)
	{
		piLock(1);
		int module_loaded = sonar_enabled;
		piUnlock(1);

		//do not trigger ultrasonic readings if disabled
		if(!module_loaded)
			continue;

		unsigned int sum = 0;
		int valid = 0;
		int i;  
		for (i = 0; i < 10; i++)  
		{  
			/* trigger reading */  
			digitalWrite(TRIGGER_PIN, HIGH);  
			waitforpin(ECHO_PIN, TIMEOUT, 10); /* wait 10 microseconds */  
			digitalWrite(TRIGGER_PIN, LOW);  
			/* wait for reading to start */  
			waitforpin(ECHO_PIN, HIGH, 5000); /* 5 ms timeout */  
			if (digitalRead(ECHO_PIN) == HIGH)  
			{  
				pulsewidth = waitforpin(ECHO_PIN, LOW, 65000L); /* 60 ms timeout */  
				if (digitalRead(ECHO_PIN) == LOW)  
				{  
					/* valid reading code */  
					//printf("echo at %d micros\n", pulsewidth);  
					sum += pulsewidth;
					valid++;
				}  
				else  
				{  
					/* no object detected code */  
					//printf("echo timed out\n"); 
					sum += 65000; //more than 65ms > INFINITY
					valid++;	   
				}  
			}  
			else  
			{  
				/* sensor not firing code */  
				//printf("sensor didn't fire\n");  
				//fail silently and ignore this reading
			}  
		}

		//if(valid==0)
		//report error

		if(valid>0)
		{
			piLock(0);
			sensor_dist = (sum/valid);
			if(sensor_dist<DIST_THRESHOLD)
				digitalWrite(SENSOR_DEBUG_PIN, HIGH); 
			piUnlock(0);
		}

	}
}

//================================================================
//  END OF MATCH TIMER
//================================================================
/* This flag controls termination of the main loop. */
volatile sig_atomic_t keep_going = 1;

/* The signal handler just clears the flag and re-enables itself. */
void catch_alarm (int sig)
{
	state = END;
	puts("END OF MATCH - TIME OUT");
	keep_going = 0;
	signal (sig, catch_alarm);
}

// Time since beginning of match
unsigned int now()
{
	return millis() - round_start_time;
}

// Time until end
unsigned int remaining()
{
	return 90000 - now();
}

//================================================================
//  STARTER - JACK DETECTOR
//================================================================
//void JackStartInterrupt (void)
//{ 
//	if(!started)//&&!checking
//	{
//	    printf(">>> JACK Interrupt detected");
//		state = CHECK_START;
//    }		
//}

void checkForStart(void)
{
    if(started)
		return;
	
	int input;
	int entries[2] = {0,0};
	int i;
	for(i=0; i<10; i++)
	{
		input = digitalRead(JACK_PIN);
		//printf("%d",input);
		entries[input]++;
		//delay(50);
	}
	
	//#if overall input is LOW jack is gone
    if (entries[0] > entries[1])
	{
		printf("Jack released");
		started = 1; //true
		state = INIT;
	}
}

int act_count;

void selectNextAction(void)
{
	piLock(2);
    int canProceed = finished;
	piUnlock(2);
	
	if(!canProceed)
		return;
		
	piLock(2);
    finished = 0;
	piUnlock(2);	
	
		printf ("Can proceed. Action %d\n", act_count);
	switch(act_count)
	{
	case ACTION_MOVE:
		puts("======= MOVE ACTION=============");
		cmd_val = 200; //mm
		int t = piThreadCreate (move) ;
		if ((t) != 0)
		{  
			fprintf (stderr, "Failed to start move.\n", strerror (errno)) ;
			exit(-1) ;  
		}
		state = ACTION;
		break;
	case ACTION_WAIT:
		puts("======= Waiting =============");
		delay(500);
		break;
	case ACTION_TURN:
		puts("======= turn ACTION=============");
		cmd_val = 180; //deg
		int t1 = piThreadCreate (turn) ;
		if ((t1) != 0)
		{  
			fprintf (stderr, "Failed to start move.\n", strerror (errno)) ;  
			exit(-1);  
		}
		state = ACTION;
		break;
		
	default:
		puts ("No more actions to do.");
		stopMotors();
		state = END;
		break;
		
	}
	act_count++;
}

void killAll() //cutoff all sys here
{
	stopMotors();
}


//                               v-------+
//state: LOAD > WAIT > INIT > STRAT > ACTION > 
void do_stuff (void)
{
	//piLock(2);
	//printf("Finished %d\n", finished );
	//printf("Dist: %dms, %dcm\n", sensor_dist, sensor_dist/58 );
	//piUnlock(2);
	//delay(500);

	switch(state)
	{
	case CHECK_START:
		//puts ("Checking for start condition.");
		checkForStart();
		break;
	case INIT:
		puts ("Start round.");
		round_start_time = millis();
		/* Setup End-of-Match alarm. */
		alarm (90);
		state = STRAT;
		break;
	case STRAT:
		//puts ("Selecting action....");
		selectNextAction();
		break;
	case ACTION:
		if(finished)
			state = STRAT;
		break;
		
	case END:
		keep_going = 0;
		killAll();
	    puts("END OF MATCH - END OF PROGRAM");
		break;
	case BLOCKED:
		break;
	case WAIT:
	default:
			break;

	}
}

int main (void)
{
	state = LOAD;
	started = 0;
	finished = 1;
	act_count=0;

	/* Establish a handler for SIGALRM signals. */
	signal (SIGALRM, catch_alarm);
	
	ENCODERS_RES = 360; //Ticks/tr
    WHEEL_RADIUS = 50;  //mm
    FOOTING = 250;      //mm (?)

    ROBOT_RADIUS = FOOTING/2;

    MMPARTICK = (2*PI*WHEEL_RADIUS)/ENCODERS_RES; //0,87266462599716478846184538424431...
    TICKPARMM = 1/MMPARTICK;
	
	puts ("Loading wiringPi....");
	/* init GPIO & time tracking */
	if (wiringPiSetup () == -1)  
	{  
		fprintf (stderr, "Failed to initialise wiringPi: %s\n", strerror (errno)) ;  
		return 1 ;  
	} 
	
	puts ("Setting GPIO Pins modes....");
	pinMode(TRIGGER_PIN, OUTPUT);  
	pinMode(ECHO_PIN, INPUT);

	pinMode(JACK_PIN, INPUT); 
	pinMode(COLOR_SELECT, INPUT);
	
	pinMode(SENSOR_DEBUG_PIN, OUTPUT); 

	puts ("Initializing sonar module....");
	/*Start sensor module*/
	int t = piThreadCreate (sonarReading) ;
	if ((t) != 0)
	{  
		fprintf (stderr, "Failed to initialise sonar module.\n", strerror (errno)) ;  
		return 1 ;  
	}  
	
	//puts ("Hooking up Jack interrupt....");
	//wiringPiISR (JACK_PIN, INT_EDGE_FALLING, &JackStartInterrupt) ;
	
	puts ("Loading MD25 I2C module....");
	if ((fd = open(fileName, O_RDWR)) < 0)
	{// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}
	if(ioctl(fd, I2C_SLAVE, address) < 0)
	{// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}		
	buf[0] = 13;  // This is the register we wish to read software version from
	if ((write(fd, buf, 1)) != 1) 
	{//Send register to read software from
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	if (read(fd, buf, 1) != 1) 
	{// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		printf("Software version: %u\n", buf[0]);
	}
	
	resetEncoders();// Reset the encoder values to 0
	
	(void)piHiPri (2) ;

	puts ("Doing stuff while waiting for alarm....");
	state = CHECK_START;
	/* Check the flag once in a while to see when to quit. */
	while (keep_going)
		do_stuff ();

	
	int y = millis();
	printf("Tps: %dms (total %d ms)",y-round_start_time, y);
	system("echo sudo reboot");
	return EXIT_SUCCESS;
}
