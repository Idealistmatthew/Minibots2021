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

// Self Implemented
// define variables
int left_sensor_state;
int right_sensor_state;

// the setup routine runs once when you press reset:
void setup()
{
    //Light Sensor setup code
    Serial.begin(9600);
    if (tcs.begin())
    {
        Serial.println("Found sensor");
    }
    else
    {
        Serial.println("No TCS34725 found ... check your connections");
        while (1)
            ;
    }

    //Servo start code
    myservo.attach(5);
    myservo.write(90); // move servos to center position -> 90°

    // IR Line Follower
    // put your setup code here, to run once:
    pinMode(leftIR, INPUT);
    pinMode(rightIR, INPUT);

    // EXTRA (self implemented)
    // Motor setup code
    pinMode(motor1, OUTPUT);
    pinMode(motor2, OUTPUT);

    // Ultrasonic Sensors setup code
    Serial.begin(9600);

    // Gyroscope setup code
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
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
    switch (mpu.getGyroRange())
    {
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
    switch (mpu.getFilterBandwidth())
    {
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

bool following_line = True;

void loop()
{
    // put your main code here, to run repeatedly:
    //Continually checks distance
    float cm = testSensor.measureDistanceCm();

    // Light Sensor
   uint16_t r, g, b, c, colorTemp, lux;
 
   tcs.getRawData(&r, &g, &b, &c);

    while (following_line)
    {

        left_sensor_state = digitalRead(LEFTIR);
        right_sensor_state = digitalRead(RIGHTIR);

        if(right_sensor_state == 0){
            Serial.println("turning right");
            right(motor1, motor2);
        }

        if(left_sensor_state == 0){
            Serial.println("turning left");
            left(motor1, motor2);
        }
    }

    //Too close to other robot
    if (cm< 10 && r < 3000){
        brake(motor1, motor2);
        standby(motor1, motor2);
    }

    forward(motor1, motor2);
    // ACTION POINT: 3000 is an arbitrary value that REQUIRES CALIBRATING
    if (r > 3000){
        brake(motor1, motor2);
        following_line = False;
        bool detect_object = False;
        while(detect_object == False){
            right(motor1,motor2, 10);
            //ACTION POINT: CALIBRATE DISTANCE OF CM
            if (cm < 10){
                detect_object = True;
                //ACTION POINT: collect_object function is NOT WRITTEN YET
                collect_object();
                following_line = True
                //ACTION POINT: return_object function is not written yet
                return_object();
            }
        }
    }
    

}

//ACTION POINT: collect_object function is NOT WRITTEN YET
void collect_object(){
    while(cm < 1){
        forward(motor1, motor2);
    }
    //ACTION POINT: VALUE OF 90 NEEDS CALIBRATING
    myservo.write(90)
    while (left_sensor_state == )
}

//ACTION POINT: collect_object function is NOT WRITTEN YET
void return_object(){

}