#include <Arduino.h> //ARDUINO MAIN INCLUDE FILE
#include <DueTimer.h> //ARDUINO DUE TIMER INTERRUPT HEADER
#include "CytronMotorDriver.h" //MOTOR DRIVER HEADER
#include <DeadReckoner.h> //DEAD RECKONING HEADER
#include <PID_v1.h>	//PID CONTROLLER HEADER
#include <math.h> // MATH ARDUINO HEADER
#include <Servo.h> // SERVO MOTOR

#define BUFFER_SIZE 108//This will prevent buffer overruns. was 32
volatile char inData[BUFFER_SIZE];//This is a character buffer where the data sent by the python script will go.
volatile char inChar=-1;//Initialie the first character as nothing
volatile int i = 0; //Arduinos are not the most capable chips in the world so I just create the looping variable once

Servo griparm;  // create servo object to control a servo
Servo gripper;  // create servo object to control a s	ervo
Servo blobarm;

// Loop Logic Variables
int LegoKey = 1; // For getting lego Keypoint once
int LegoCount = 0;
volatile int LegoAngleOn = 0;
volatile int LegoDropAngleOn = 0;
volatile int HomeAngleOn = 0;
volatile int TurnAngleOn = 0;
volatile float RobotX = 0;
volatile float RobotY = 0;
volatile float RobotX2 = 0;
volatile float RobotY2 = 0;
volatile float OpenCV[5]; // changed to volatile during dpoint calib
volatile float desiredAngle = 0;
float dist_to_LegoWaypoint = 0; // Initialize distance to waypoint to determine if near planned point
float dist_to_DropWaypoint = 0; // Initialize distance to waypoint to determine if near planned point
float dist_to_HomeWaypoint = 9999; // Initialize distance to waypoint to determine if near planned point
volatile double currentAngle = 0;
volatile float angleError = 0; // Initialize angle error var to store difference between current and required.

float LegoX = 9999; 
float LegoY = 9999;
float DropX = 9999;
float DropY = 9999;
float HomeX = 9999;
float HomeY = 9999;

//*BLUETOOTH INIT (TX & RX)
#define HC05 Serial1

//*ENCODER PINS on D Port of Due. //still need this because pid dep. on angular vel
#define ENCODER_LEFT_PINA 28 
#define ENCODER_LEFT_PINB 27
#define ENCODER_RIGHT_PINA 26
#define ENCODER_RIGHT_PINB 25

//*MEASUREMENTS
#define RADIUS 31 // Wheel radius in mm
#define LENGTH 137 // Wheel base length in mm
#define TICKS_PER_REV 5880 //Quadrature Encoder Pulses per Revolution

//*MOTOR INIT
CytronMD motor1(PWM_DIR, 5, 4);
CytronMD motor2(PWM_DIR, 6, 7);

//*CONST RATE FOR TURNING
#define ANGULAR_RATE 0.9 // was 0.9 on good runs

//*Initialize tick values to 0
volatile long leftTicks = 0, rightTicks = 0;

//LEFT MOTOR QUADRATURE ENCODER PULSE INCREMENT
//HARDWARE INTERRUPT PINS 27 AND 28 	
void pulseLeft() { 
	static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	static uint8_t enc_val1 = 0;

	enc_val1 = enc_val1 << 2;
	enc_val1 = enc_val1 | ((REG_PIOD_PDSR & 0b00001100) >>2);
	
	leftTicks = leftTicks + lookup_table[enc_val1 & 0b00001111]; 
}

//RIGHT MOTOR QUADRATURE ENCODER PULSE INCREMENT
//HARDWARE INTERRUPT PINS 25 AND 26
void pulseRight() {
	static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
	static uint8_t enc_val2 = 0;

	enc_val2 = enc_val2 << 2;
	enc_val2 = enc_val2 | (REG_PIOD_PDSR & 0b00000011);
	
	rightTicks = rightTicks + lookup_table[enc_val2 & 0b00001111]; 
}

//*Intialize constructor for dead reckoner function use 
// This is going to give me angular velocity and angle only now
DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);

//*PID variable init
volatile double motorspeed1 = 0, motorspeed2 = 0; // Motor speed for left and right respectively 
volatile double forwards_vel = 0, angular_vel= 0; // Forward velocity and angular velocity set points
volatile double Setpoint1 = 0, Input1 = 0, Output1 = 0;	// Arguments to pass in motor 1 PID controller
volatile double Setpoint2 = 0, Input2 = 0, Output2 = 0; // Arguments to pass in motor 2 PID controller

//*PID gains for motor 1
double Kp1=3.7, Ki1=9.2, Kd1=0.05; 
//*PID gains for motor 2
double Kp2=3.7, Ki2=9.2, Kd2=0.04; 

//*Initialize PID constructor for motor 1 (left motor)
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, P_ON_E); // Passes addresses of variables initalized and gains
//*Initialize PID constructor for motor 2 (right motor)
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, P_ON_E); // Passes addresses of variables initalized and gains

//*Function used to convert angular velocity to required PWM motor 1
void speedToPWM1(double output){
	if (output > 0){ // If required speed is greater than 0
		motorspeed1 = (3.9592*sq(output)) - (1.123*(output)); 
	}

	if (output < 0){  // If required speed is less than 0
		motorspeed1 = (-3.9592*sq(output)) - (1.123*(output)); 
	}
}

//*Function used to convert angular velocity to required PWM motor 2
void speedtoPWM2(double output){
	if (output > 0){ // If required speed is greater than 0
		motorspeed2 = (2.5224)*sq(output) + (12.405)*(output);
	}
	if (output < 0){ // If required speed is less than 0
		motorspeed2 = (-2.5224)*sq(output) + (12.405)*(output);
	}
}

//*Method to closest direction to turn to reach angle 
void direction (volatile float angleError){
	if ((angleError)<=0){ //*If difference is less than zero, (negative rotation - clockwise)
		forwards_vel = 0;	// No forward velocity
		angular_vel = -ANGULAR_RATE; // Negative angular rate, to turn clockwise
	}

	else{ //*Positive rotation, counter clockwise 
		forwards_vel = 0;	// No forward velocity 
		angular_vel = ANGULAR_RATE;	// Positive angular velocity, to turn counter clockwise 
	}
}

void pickupLego(){
  	gripper.write(0);   // close gripper 
  	delay(1000);	    // allow 1 to close the gripper 
  	griparm.write(85);  // bring the arm back up with the lego
	delay(2000);
}

void dropLego(){
	griparm.write(50);  // bring gripper arm down was 45
	delay(2000);	    // allow 3 seocnds for arm to reach down position
	gripper.write(180); // open gripper arm 
	delay(2000);        // wait 1 second before 	
}

String convertToString(volatile char* a, String variable) { // take character array inuput, seperate coordinates
  int x; // start of byte
  int size; // end of byte

    if (variable == "LX"){ // lego x coordinate, 1-7 bytes
      x = 1;
      size = 7; 
    }
    if (variable == "LY"){ // lego y coordinate, 10-16 bytes
      x = 10;
      size = 16; 
    }
    if (variable == "RX"){ // Robot x coordinate, 19-25 bytes
      x = 19;
      size = 25; 
    }
    if (variable == "RY"){ // Robot y coordinate, 28-34 bytes
      x = 28;
      size = 34; 
    }

    if (variable == "CA"){ // Current robot heading in deg
      x = 37;
      size = 43; 
    }

    String s = ""; // convert character array to string
    for (i=x ; i < size; i++) { // increment character by number of bytes
        s = s + a[i]; 
    } 
    return s; // return string for respective coordinate
} 

void cvpoints(){
	byte byte_count=Serial1.available();//This gets the number of bytes that were sent by the python script
	
	//If there are any bytes then deal with them
	if(byte_count){
		int first_bytes=byte_count;//initialize the number of bytes that we might handle. 
		int remaining_bytes=0;//Initialize the bytes that we may have to burn off to prevent a buffer overrun
			
			if(first_bytes>=BUFFER_SIZE-1){ //If the incoming byte count is more than our buffer...
				remaining_bytes=byte_count-(BUFFER_SIZE-1); //Reduce the bytes that we plan on handleing to below the buffer size
			}
			
			for(i=0;i<first_bytes;i++){//Handle the number of incoming bytes
				inChar=Serial1.read();//Read one byte
				inData[i]= inChar;//Put it into a character string(array)
			}

			String sx = convertToString(inData, "LX"); // lego x and convert to string and store in sx1
    		String sy = convertToString(inData, "LY"); // robot x and convert to string and store in sx2
    		String sx1 = convertToString(inData, "RX"); // robot x2 and convert to string and store in sx2
    		String sy1 = convertToString(inData, "RY"); // lego y and convert to string and store in sy1
    		String sx2 = convertToString(inData, "CA"); // robot y and convert to string and store in sy2

    		OpenCV[0]= sx.toFloat(); // convert string to float & store lego x in index 0 of open cv array
    		OpenCV[1]= sy.toFloat(); // convert string to float & store lego y in index 1 of open cv array
    		OpenCV[2]= sx1.toFloat(); // convert string to float & store robot x1 in index 2 of open cv array
    		OpenCV[3]= sy1.toFloat(); // convert string to float & store robot y1 in index 3 of open cv array
    		OpenCV[4]= sx2.toFloat(); // convert string to float & store robot x2 in index 4 of open cv array
		
			for(i=0;i<remaining_bytes;i++){//This burns off any remaining bytes that the buffer can't handle.
				inChar=Serial1.read();
			}
	
    	inData[i]='\0';//This ends the character array with a null character. This signals the end of a string  
	}
}

void CurrentPose() {
	cvpoints();
  	RobotX = OpenCV[2];
  	RobotY = OpenCV[3]; 
	
	// Retrieve current angle from OpenCV
	currentAngle = OpenCV[4];	

	// Can optimize deadreckoning for angular vel and angle
	deadReckoner.computePosition(); // Computes current pose of robot

	if(LegoAngleOn == 1){ 
		desiredAngle = ((atan2(LegoY - RobotY, LegoX - RobotX))*RAD_TO_DEG) - 90;
	}

	if(LegoDropAngleOn == 1){
		desiredAngle = ((atan2(DropY - RobotY, DropX - RobotX))*RAD_TO_DEG) - 90;
	}
	
	if(HomeAngleOn == 1){
		desiredAngle = ((atan2(HomeY - RobotY, HomeX - RobotX))*RAD_TO_DEG) - 90;
	}
	
	if (desiredAngle <= -180) {
		desiredAngle = desiredAngle + 360;
	}

	angleError = desiredAngle - currentAngle;

	if(TurnAngleOn == 1){
		//*If absolute value of angle differnce between current angle and
		// waypoint angle is less than 3 degrees, continue forward motion
		if ((abs(angleError))<=8){ // was 2.5
			forwards_vel = 5; // Forward Angular Vel = 5 rad/s
			angular_vel = 0;  // No Rotation 
		}

		// If absolute value of angle difference between current angle and 
		// way point angle exceeds 3 degrees, call direction method and apply angular rotation
		if ((abs(angleError))>8){
			direction(angleError);
		}
	}

 	double wl = deadReckoner.getWl(); // Returns current angular velocity of left wheel
	double wr = deadReckoner.getWr(); // Returns current angular velocity of right wheel

	Input1 = wl;	// Set PID for motor 1 input to current left angular velocity
	Input2 = wr;	// Set PID for motor 2 input to current right angular velocity

	Setpoint1 = forwards_vel - angular_vel; // Setpoint formula for motor 1 desired speed
	Setpoint2 = forwards_vel + angular_vel; // Setpoint formula for motor 2 desired speed

	myPID1.Compute(); // Computes error between current motor 1 speed and desired speed 
	myPID2.Compute(); // Computes error between current motor 2 speed and desired speed

	speedToPWM1(Output1); // Convert output speed error from myPID1 to PWM 
	speedtoPWM2(Output2); // Convert output speed error from myPID2 to PWM 

	motor1.setSpeed(motorspeed1); // Set motor 1 to converted PWM
	motor2.setSpeed(motorspeed2); // Set motor 2 to converted PWM
}

void attachInterrupts() {
	//*Set up hardware interrupts for both Phase A and Phase B of motors 1 and 2
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PINA), pulseLeft, CHANGE); // Call pulseLeft on phase A change
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PINB), pulseLeft, CHANGE); // Call pulseLeft on phase B change
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PINA), pulseRight, CHANGE); // Call pulseRight on phase A change
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PINB), pulseRight, CHANGE); // Call pulseRight on phase A change
	Timer1.attachInterrupt(CurrentPose);
	Timer1.start(20000); // 20ms frequency timer interrupt
}

void setup() {
	//*Set encoder pin mode to input pullup
	pinMode(ENCODER_LEFT_PINA, INPUT_PULLUP); 
	pinMode(ENCODER_LEFT_PINB, INPUT_PULLUP);
	pinMode(ENCODER_RIGHT_PINA, INPUT_PULLUP);
	pinMode(ENCODER_RIGHT_PINB, INPUT_PULLUP);
	Serial.begin(115200); // Initilalize serial monitor baud rate
	Serial1.begin(115200); 	// Initialize HC05 bluetooth module
	attachInterrupts(); // Attach hardware and timer interrupt
	myPID1.SetMode(AUTOMATIC);	//Initialize PID controller for motor 1
	myPID2.SetMode(AUTOMATIC);	//Initialize PID controller for motor 2
	pinMode(51, OUTPUT); // Initilize gripper arm pinmode as output
	griparm.attach(51, 500, 2300);   // attaches the servo on pin 51 to the servo object - griparm
    gripper.attach(A0); // attaches the servo on pin A0 to the servo object - gripper mechanism
	blobarm.attach(A2); // attaches the servo on pin A2 to the servo object - blob mechanism
	blobarm.write(0); // Set the blob arm to 0
}

void loop() {
	if(millis() < 5000){
		//This helps initial retrieval of coordinates rather than passing what the vars were init to
		gripper.write(180); // open gripper arm 	
		delay(1000);
		griparm.write(105);
		delay(1000);
		griparm.write(36);
		delay(2000);
		blobarm.write(100);
		delay(1000);
	}

	// Retrieve Lego KeyPoint
	if(LegoKey == 1){
		// Need to store in global var of lego location
		LegoX = OpenCV[0];
		LegoY = OpenCV[1];
		LegoAngleOn = 1;
		LegoKey = 0;
		TurnAngleOn = 1;
		delay(4500); 
		griparm.write(36);
	}

	dist_to_LegoWaypoint = sqrt((pow(LegoX - RobotX, 2)) + (pow(LegoY -  RobotY, 2)));
	dist_to_DropWaypoint = sqrt((pow(DropX - RobotX, 2)) + (pow(DropY -  RobotY, 2)));
	dist_to_HomeWaypoint = sqrt((pow(HomeX - RobotX, 2)) + (pow(HomeY -  RobotY, 2)));

	if (dist_to_DropWaypoint < 150){ 
		TurnAngleOn = 0;
		forwards_vel = 0;
		angular_vel = 0;
		LegoDropAngleOn = 0;
		
		DropX = 9999;
		DropY = 9999;
		dropLego();
		delay(1000);
		LegoKey = 1;
		LegoCount = LegoCount + 1;
	}

	if(LegoCount>= 4){
		LegoKey = 0;
		HomeX = 80; 
		HomeY = 80;	
		HomeAngleOn = 1;
		TurnAngleOn = 1;
		griparm.write(105);	
	}
	
	if (dist_to_LegoWaypoint < 95){ 
		TurnAngleOn = 0;
		forwards_vel = 0; 
		angular_vel = 0;  
		LegoAngleOn = 0;
		LegoX = 9999;
		LegoY = 9999;
		delay(1000); 
		pickupLego();

		if (LegoCount < 2){
			DropX = 1430; // set drop location 
			DropY = 56; // set drop location
		}

		else{
			DropX = 1410; // set drop location 
			DropY = 1935; // set drop location
		}
		
		LegoDropAngleOn = 1;
		TurnAngleOn = 1;
	}

	if (dist_to_HomeWaypoint <= 15){  
		HomeAngleOn = 0;
		TurnAngleOn = 0;
		LegoCount = 0;
		forwards_vel = 0; 
		angular_vel = 0; 
		blobarm.write(0); // set to zero position
		delay(1000);
		Timer1.detachInterrupt();
	}
}	

