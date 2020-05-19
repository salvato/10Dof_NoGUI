#pragma once
#include <inttypes.h>

class MotorController_L298
{
public:
    MotorController_L298(uint32_t _ena, uint32_t _in1, uint32_t _in2,
                         uint32_t _enb, uint32_t _in3, uint32_t _in4,
                         double _motorAConst, double _motorBConst);
    ~MotorController_L298();
    void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
    void move(int speed);
    void move(int speed, int minAbsSpeed);
    void turnLeft(int speed, bool kick);
    void turnRight(int speed, bool kick);
    void stopMoving();

protected:
    void initPins();
    int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

protected:
    int32_t  gpioHostHandle;
    uint32_t pwm1Pin, mot1in1Pin, mot1in2Pin;
    uint32_t pwm2Pin, mot2in1Pin, mot2in2Pin;
    uint32_t PWMfrequency;
    uint32_t dutyCycle;
    int32_t  currentSpeed;
    double   motor1Const;
    double   motor2Const;
};

