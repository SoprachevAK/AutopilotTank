
#include "GyverMotor.h"
#include <ServoSmooth.h>
#include "I2Cdev.h"
#include <VL53L1X.h>

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

const int LEFT_ENC_PIN = 2;
const int RIGHT_ENC_PIN = 3;
const int SERVO_PIN = 12;

bool printQuat = false;
bool printDist = false;
bool printEnc = false;

GMotor motorR(DRIVER2WIRE, 7, 5, HIGH);
GMotor motorL(DRIVER2WIRE, 4, 6, HIGH);
VL53L1X lox;
ServoSmooth servo;
MPU6050 mpu = MPU6050();

int leftCount = 0;
int rightCount = 0;

bool mpuEnable = false;
uint8_t mpuIntStatus = 0;
uint16_t packetSize = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;

int targetSpeed = 0;
int targetRot = 0;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

int delta = 10;

static int sing(int val)
{
    if (val > 0)
        return 1;
    if (val < 0)
        return -1;
    return 0;
}

static bool compareStr(String s1, String s2, int from, int to)
{
    if (to <= from)
    {
        return false;
    }

    int d = to - from;
    if (s1.length() < to || s2.length() < d)
    {
        return false;
    }

    for (int i = from; i < to; i++)
    {
        if (s1[i] != s2[i - from])
        {
            return false;
        }
    }

    return true;
}

void leftEncInterrupt()
{
    leftCount += sing(currentLeftSpeed);
}

void rightLeftInterrupt()
{
    rightCount += sing(currentRightSpeed);
}

void setup()
{
    Serial.begin(115200);
    setupServo();
    setupMotor();
    setupEnc();
    setupLOX();
    setupMPU();

    delay(100);
    Serial.println("DONE");
}

void setupServo()
{
    servo.smoothStart();
    servo.attach(SERVO_PIN, 600, 2400);
    servo.setSpeed(50);
    servo.setAccel(0.3);

    servo.setAutoDetach(false);

    servo.setTargetDeg(90);

    while (abs(90 - servo.getCurrentDeg()) > 2)
    {
        servo.tick();
        delay(10);
    }
    servo.setTargetDeg(0);
}

void setupLOX()
{
    lox.setTimeout(500);
    while (!lox.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        delay(100);
    }
    lox.setDistanceMode(VL53L1X::Long);
    lox.setMeasurementTimingBudget(50000);
    lox.startContinuous(50);
}

void setupEnc()
{
    pinMode(LEFT_ENC_PIN, INPUT);
    pinMode(RIGHT_ENC_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightLeftInterrupt, FALLING);
}

void setupMPU()
{
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();

    int devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        mpu.CalibrateAccel(20);
        mpu.CalibrateGyro(20);

        mpu.setDMPEnabled(true);

        //attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        mpuEnable = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setupMotor()
{
    motorR.setMode(FORWARD);
    motorL.setMode(FORWARD);

    motorR.setMinDuty(100);
    motorL.setMinDuty(100);

    setSpeed(0, 0);
}

long time = 0;
bool val = false;
void loop()
{
    if (millis() - time > 500)
    {
        time = millis();
        digitalWrite(13, val);
        val = !val;
    }
    encTick();
    servoTick();
    loxTick();
    mpuTick();
    serialRead();
}

void serialRead()
{
    static String s = "";
    static bool end = false;
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == ';')
            end = true;
        else
            s += c;
    }

    if (end)
    {
        if (s == "quat 1")
        {
            printQuat = true;
        }
        else if (s == "quat 0")
        {
            printQuat = false;
        }
        else if (s == "dist 1")
        {
            printDist = true;
        }
        else if (s == "dist 0")
        {
            printDist = false;
        }
        else if (s == "enc 1")
        {
            printEnc = true;
            printEncState();
        }
        else if (s == "enc 0")
        {
            printEnc = false;
        }
        else
        {
            if (compareStr(s, "move", 0, 4))
            {
                targetSpeed = constrain(s.substring(5).toInt(), -255, 255);
                targetRot = 0;
                updSpeed();
            }
            else if (compareStr(s, "rot", 0, 3))
            {
                targetSpeed = 0;
                targetRot = constrain(s.substring(4).toInt(), -255, 255);
                updSpeed();
            }
            else if (compareStr(s, "setDelta", 0, 8))
            {
                delta = constrain(s.substring(8).toInt(), -255, 255);
                Serial.println("delta updated");
            }
        }

        s = "";
        end = false;
    }
}

void updSpeed()
{
    if (targetSpeed != 0)
    {
        setSpeed(targetSpeed - delta, targetSpeed + delta);
    }
    else if (targetRot != 0)
    {
        if (targetRot < 0)
        {
            setSpeed(0, abs(targetRot) + delta);
        }
        else
        {
            setSpeed(abs(targetRot), 0);
        }
    }
    else
    {
        setSpeed(0, 0);
    }
}

void servoTick()
{
    servo.tick();
    if (servo.getCurrentDeg() > 178)
    {
        servo.setTargetDeg(0);
    }
    else if (servo.getCurrentDeg() < 2)
    {
        servo.setTargetDeg(180);
    }
}

void encTick()
{
    static int oldLeft = 0;
    static int oldRight = 0;
    if (oldLeft != leftCount || oldRight != rightCount)
    {
        oldRight = rightCount;
        oldLeft = leftCount;
        printEncState();
    }
}

void loxTick()
{
    if (lox.dataReady())
    {
        int16_t dist = lox.read();
        if (printDist)
        {
            Serial.print("dist ");
            Serial.println(dist);
        }
    }
}

void mpuTick()
{
    static VectorFloat gravity;
    static float ypr[3];
    if (!mpuEnable)
        return;

    if (currentRightSpeed != 0 || currentLeftSpeed != 0)
    {
        return;
    }

    while (fifoCount < packetSize)
    {
        fifoCount = mpu.getFIFOCount();
    }

    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize)
    {
    }
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
    }
    else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        while (fifoCount >= packetSize)
        {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if (printQuat)
        {
            Serial.print("ypr ");
            Serial.print(ypr[0] * 180 / M_PI);
            Serial.print(" ");
            Serial.print(ypr[1] * 180 / M_PI);
            Serial.print(" ");
            Serial.println(ypr[2] * 180 / M_PI);
        }
    }
}

void printEncState()
{
    if (printEnc)
    {
        Serial.print("enc ");
        Serial.print(leftCount);
        Serial.print(" ");
        Serial.println(rightCount);
    }
}

void setSpeed(int leftSpeed, int rightSpeed)
{
    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;
    Serial.print("Set speed {");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.print(rightSpeed);
    Serial.println("}");
    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);
}