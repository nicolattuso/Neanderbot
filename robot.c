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

#define ENCODERS_RES 360 //Ticks/tr
#define WHEEL_RADIUS 50  //mm
#define FOOTING 250      //mm (?)
#define ROBOT_RADIUS (FOOTING/2)
#define MMPARTICK ((2*PI*WHEEL_RADIUS)/ENCODERS_RES) //0,87266462599716478846184538424431...
#define TICKPARMM (1/MMPARTICK)
#define MMTOSTEP(mm) ((mm)*TICKPARMM)
#define DEG2RAD(x) ((x)*(PI/180))
#define RAD2DEG(x) ((x)*(180/PI))

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

#define DIST_THRESHOLD 10 //cm

#define MD25ADDRESS 0x58          // Address of MD25 shifted one bit

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


//=========================================
/// structs
//=========================================

typedef enum RobState 
{ 
	LOAD, WAIT, INIT, CHECK_START, STRAT, ACTION, BLOCKED, RECUP, END 
}RobState;

typedef enum TaskState {
	TASK_INIT,
	TASK_RUNNING,
	TASK_DONE
} TaskState;

typedef enum TaskType {
	TASK_CHECK_START_CONDITION,
	TASK_MOVE,
	TASK_TURN,
	TASK_SONAR
} TaskType;

typedef struct Task {
	TaskState (*task)(void*);
	TaskState state;
	TaskType type;
	struct Task* next;
	struct Task* previous;
	void* private;
} Task;

typedef enum ActionType {
	ACTION_MOVE,
	ACTION_WAIT,
	ACTION_TURN,
	ACTION_STOP,
	NUM_ACTIONS
} ActionType;


//=========================================
/// globals
//=========================================
RobState state = LOAD;
int sensor_dist = 9999;
int fd; //i2c file descriptor
char *fileName = "/dev/i2c-1";
double m_c1,m_c2;

//=========================================
/// functions
//=========================================



/* The signal handler just clears the flag and re-enables itself. */
void catch_alarm (int sig)
{
	state = END;
	signal (sig, catch_alarm);
}

void resetEncoders(void)
{
	char buf[10];
	buf[0] = 16; // Command register
	buf[1] = 32; // command to set decoders back to zero
	if ((write(fd, buf, 2)) != 2)
	{
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

void stopMotors(void)
{ 
	char buf[10];
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

TaskState checkForStart(void* private)
{	
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
		return TASK_DONE;
	} else {
		return TASK_RUNNING;
	}
}

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

TaskState sonarReading(void* private)
{
	int state = 0 ;
	int debounceTime = 0 ;
	int pulsewidth; 


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
				delay(60);
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
		sensor_dist = (sum/valid);
		if((sensor_dist/58)<DIST_THRESHOLD)
			digitalWrite(SENSOR_DEBUG_PIN, HIGH); 
		else
			digitalWrite(SENSOR_DEBUG_PIN, LOW); 
	}
	return TASK_RUNNING;
}

long readEncoderValues (void)
{
	char buf[10];
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
	char buf[10];
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

TaskType move(void* private)
{
	int cmd_val = *((int*)(private));
	double ticks = MMTOSTEP(cmd_val);
	
	double sonarEcho;
	
    if(readEncoderValues() < ticks)
    { // Check the value of encoder 1 and stop after it has traveled a set distance
		sonarEcho = sensor_dist/58;
		
		if(sonarEcho < DIST_THRESHOLD)
		{
			puts("Obstacle !!!");
			stopMotors();
			return TASK_RUNNING;
		}
		else {
			driveMotors();
			return TASK_RUNNING;
		}
    } else {
		stopMotors();
		return TASK_DONE;
	}
	//should not happen
	return TASK_DONE;
}

void turnMotors(int angle)
{
	char buf[10];
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

TaskType turn(void* private)
{
	int cmd_val = *((int*)(private));
	printf ("turn Enter.(%d mm)", cmd_val);
	resetEncoders();
	
	double consign = MMTOSTEP(ROBOT_RADIUS * DEG2RAD(cmd_val));
	
    //set motors to opposite speeds while traveled arc < consign
    if(readEncoderValues() < consign)
    { // Check the value of encoder 1 and stop after it has traveled a set distance
        turnMotors(consign);
        return TASK_RUNNING;
    } else {
		stopMotors();
		return TASK_DONE;
	}
	return TASK_DONE;
}

void append_task(Task* current, Task* new) {
	if((current->next == NULL) && (current->previous == NULL)) {
		current->next=new;
		current->previous=new;
		new->next=current;
		new->previous=current;
	} else {
		current->next->previous=new;
		new->next=current->next;
		current->next=new;
		new->previous=current;
	}
}

RobState selectNextAction(Task* current_task)
{
	static int act_count;
	int * cmd_val;
	printf ("Can proceed. Action %d\n", act_count);
	switch(act_count)
	{
	case ACTION_MOVE:
		puts("======= MOVE ACTION=============");
		cmd_val = malloc(sizeof(int));
		*cmd_val = 200; //mm
		Task* move_task = malloc(sizeof(Task));
		move_task->task = sonarReading;
		move_task->state = TASK_INIT;
		move_task->type = TASK_MOVE;
		move_task->next = NULL;
		move_task->previous = NULL;
		move_task->private = (void*) cmd_val;
		append_task(current_task, move_task);
		return ACTION;
	case ACTION_WAIT:
		puts("======= Waiting =============");
		delay(500);
		break;
	case ACTION_TURN:
		puts("======= turn ACTION=============");
		cmd_val = malloc(sizeof(int));
		*cmd_val = 180; //deg
		Task* turn_task = malloc(sizeof(Task));
		turn_task->task = sonarReading;
		turn_task->state = TASK_INIT;
		turn_task->type = TASK_TURN;
		turn_task->next = NULL;
		turn_task->previous = NULL;
		turn_task->private = (void*) cmd_val;
		append_task(current_task, turn_task);
		return ACTION;
		
	default:
		puts ("No more actions to do.");
		stopMotors();
		return END;
	}
	act_count++;
}

int main (void)
{
	int started = 0;
	int act_count=0;
	unsigned int round_start_time;
	int y;
	
	/* Establish a handler for SIGALRM signals. */
	signal (SIGALRM, catch_alarm);
	
	/* init GPIO & time tracking */
	if(wiringPiSetup() == -1)
	{
		fprintf(stderr, "Failed to initialise wiringPi: %s\n", strerror (errno));
		return 1;
	}
	
	/* set GPIO mode */
	pinMode(TRIGGER_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);

	pinMode(JACK_PIN, INPUT);
	pinMode(COLOR_SELECT, INPUT);
	
	pinMode(SENSOR_DEBUG_PIN, OUTPUT);
	
	/* initialize MD25 */
	if ((fd = open(fileName, O_RDWR)) < 0)
	{// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}
	if(ioctl(fd, I2C_SLAVE, MD25ADDRESS) < 0)
	{// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}		
	char buf[10];
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
	
	/* start main loop */
	state = CHECK_START;
	Task* first_task = malloc(sizeof(Task));
	first_task->task = checkForStart;
	first_task->state = TASK_INIT;
	first_task->type = TASK_CHECK_START_CONDITION;
	first_task->next = NULL;
	first_task->previous = NULL;
	first_task->private = NULL;
	Task* current_task = first_task;
	Task* sonar_task = malloc(sizeof(Task));
	sonar_task->task = sonarReading;
	sonar_task->state = TASK_INIT;
	sonar_task->type = TASK_SONAR;
	sonar_task->next = NULL;
	sonar_task->previous = NULL;
	sonar_task->private = NULL;
	append_task(current_task, sonar_task);
	TaskState current_task_state = current_task->state;
	while(state != END) {
		/* executes a task */
		if(current_task != NULL) {
			current_task->state = current_task->task(current_task->private);
		}
		
		/* manage task list if current task is done */
		if(current_task->state == TASK_DONE) {
			if(current_task->next != NULL) {
				if(current_task->next == current_task->previous){
					current_task->next->previous = NULL;
					current_task->next->next = NULL;
				} else {
					current_task->previous->next = current_task->next;
					current_task->next->previous = current_task->previous;
				}
			} else {
				//all tasks done
				printf("all tasks done\n");
				state = END;
			}
		}
		
		/* perform actions related to the global state of the robot */
		switch(state) {
			case LOAD:
				printf("this should be impossible\n");
				return 1;
			case WAIT:
				break;
			case INIT:
				round_start_time = millis();
				alarm (90);
				state = STRAT;
				break;
			case CHECK_START:
				if((current_task->type == TASK_CHECK_START_CONDITION) && (current_task->state == TASK_DONE)) {
					state = INIT;
				}
				break;
			case STRAT:
				state = selectNextAction(current_task);
				break;
			case ACTION:
				if(((current_task->type == TASK_MOVE) || (current_task->type == TASK_TURN)) && (current_task->state == TASK_DONE)) {
					state = STRAT;
				}
				break;
			case BLOCKED:
				break;
			case RECUP:
				break;
			case END:
				stopMotors();
				goto end_of_program;
				break;
				
		}
		
		/* check if there is something to do next */
		if(state != END) {
			if(current_task->next != NULL) {
				current_task = current_task->next;
			}
		}
	}
	
end_of_program:
	y = millis();
	printf("Tps: %dms (total %d ms)",y-round_start_time, y);
	system("echo sudo reboot");
	return EXIT_SUCCESS;
}









