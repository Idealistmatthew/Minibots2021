// Import Libraries
#include <SparkFun_TB6612.h>
#include <HCSR04.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"


// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Define pins
#define echoPin A0
#define trigPin A1
#define leftIR A2
#define rightIR A3

// Pins for all inputs for the motor drivers, keep in mind the PWM defines must be on PWM pins
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// Instatiate the components
Servo myservo;
Adafruit_MPU6050 mpu;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
UltraSonicDistanceSensor testSensor(triPin, echoPin);
Adafruit_TCS34725 tcs = Adafruit_TCS34725();

// the setup routine runs once when you press reset:
void setup() {
  //Light Sensor setup code
  Serial.begin(9600);
  if (tcs.begin()) {
  Serial.println("Found sensor");
  } else {
  Serial.println("No TCS34725 found ... check your connections");
  while (1);
  }

  
  //Servo start code
  myservo.attach(5);
  myservo.write(90);// move servos to center position -> 90°
  
  // IR Line Follower
  // put your setup code here, to run once:
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  
  // Ultrasonic Sensors setup code
  Serial.begin(9600);

  // Gyroscope setup code
    Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  //I'm not sure what Jie Rui means by it being hard to implement but I'll use this as a test run for movement for now and have a separate file using the gyroscope to quantify the error in direction headed
  forward(motor1, motor2, 150);
  back(motor1, motor2, 150);
  left(motor1, motor2, 150);
  right(motor1, motor2, 150);
  forward(motor1, motor2, 150);
  brake(motor1,motor2);
  standby(motor1, motor2);
}

// The following code is here for standby just in case the motor can't run without this code. This is because I think this is code we accidentally took from the header from the package
// The code is currently commented

// // Constructor. Mainly sets up pins.
// Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin);      

// // Drive in direction given by sign, at speed given by magnitude of the 
// //parameter.
// void drive(int speed);  

// // drive(), but with a delay(duration)
// void drive(int speed, int duration);  

// //currently not implemented
// //void stop();           // Stop motors, but allow them to coast to a halt.
// //void coast();          // Stop motors, but allow them to coast to a halt.

// //Stops motor by setting both input pins high
// void brake(); 

// //set the chip to standby mode.  The drive function takes it out of standby 
// //(forward, back, left, and right all call drive)
// void standby(); 

// //Takes 2 motors and goes forward, if it does not go forward adjust offset 
// //values until it does.  These will also take a negative number and go backwards
// //There is also an optional speed input, if speed is not used, the function will
// //use the DEFAULTSPEED constant.
// void forward(Motor motor1, Motor motor2, int speed);
// void forward(Motor motor1, Motor motor2);

// //Similar to forward, will take 2 motors and go backwards.  This will take either
// //a positive or negative number and will go backwards either way.  Once again the
// //speed input is optional and will use DEFAULTSPEED if it is not defined.
// void back(Motor motor1, Motor motor2, int speed);
// void back(Motor motor1, Motor motor2);

// //Left and right take 2 motors, and it is important the order they are sent.
// //The left motor should be on the left side of the bot.  These functions
// //also take a speed value
// void left(Motor left, Motor right, int speed);
// void right(Motor left, Motor right, int speed);

// //This function takes 2 motors and and brakes them
// void brake(Motor motor1, Motor motor2);