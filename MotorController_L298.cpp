#include "MotorController_L298.h"
#include "pigpiod_if2.h" // The library to use GPIO pins on Raspberry


#include <algorithm> // min() & max()
#include "string.h"  // for memset()
#include <QDebug>
#include <QThread>

using namespace std; // min() & max()


#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1


//==============================================================================
//    43A High Power BTS7960 DC Motor Driver Module. http://handsontec.com
//==============================================================================
//     Pin No         Function                         Description
//------------------------------------------------------------------------------
//        1             RPWM            Forward Level or PWM signal, Active High
//        2             LPWM            Reverse Level or PWM signal, Active High
//        3             R_EN            Forward Drive Enable Input, Active High
//        4             L_EN            Reverse Drive Enable Input, Active High
//        5             R_IS            Forward Drive, Side current alarm output
//        6             L_IS            Reverse Drive, Side current alarm output
//        7             Vcc             +5V Power Supply Output
//        8             Gnd             Ground: Connect to microcontroller GND
//==============================================================================

//========================================
//              L298 Module:
//========================================
// i1 = High ; i2 = Low    Forward
// i1 = Low  ; i2 = High   Reverse
// i1 = i2                 Fast Motor Stop
// ena                     PWM Input
//========================================


MotorController_L298::MotorController_L298(uint32_t _ena, uint32_t _in1, uint32_t _in2,
                                 uint32_t _enb, uint32_t _in3, uint32_t _in4,
                                 double _motor1Const, double _motor2Const)
{
    pwm1Pin    = _ena;
    mot1in1Pin = _in1;
    mot1in2Pin = _in2;

    pwm2Pin    = _enb;
    mot2in1Pin = _in3;
    mot2in2Pin = _in4;

    motor1Const = _motor1Const;
    motor2Const = _motor2Const;

    //===========================================================================
    // Each GPIO can be independently set to one of 18 different PWM frequencies.
    // The selectable frequencies depend upon the sample rate which may be
    // 1, 2, 4, 5, 8, or 10 microseconds (default 5).
    // The sample rate is set when the pigpio daemon is started (Default 5).
    //    The frequencies for each sample rate are:
    //===========================================================================
    //                                 Hertz
    //---------------------------------------------------------------------------
    //           1: 40000 20000 10000 8000 5000 4000 2500 2000 1600
    //               1250  1000   800  500  400  250  200  100   50
    //
    //           2: 20000 10000  5000 4000 2500 2000 1250 1000  800
    //                625   500   400  250  200  125  100   50   25
    //
    //           4: 10000  5000  2500 2000 1250 1000  625  500  400
    //                313   250   200  125  100   63   50   25   13
    //    sample
    //     rate
    //     (us)  5:  8000  4000  2000 1600 1000  800  500  400  320
    //                250   200   160  100   80   50   40   20   10
    //
    //           8:  5000  2500  1250 1000  625  500  313  250  200
    //                156   125   100   63   50   31   25   13    6
    //
    //          10:  4000  2000  1000  800  500  400  250  200  160
    //                125   100    80   50   40   25   20   10    5
    //===========================================================================

    PWMfrequency    = 1000; // in Hz

    gpioHostHandle = pigpio_start((char*)"localhost", (char*)"8888");
    if(gpioHostHandle < 0) {
        qDebug() << QString("Unable to initialize GPIO");
        exit(EXIT_FAILURE);
    }

    // set_PWM_frequency() returns the numerically closest frequency if OK
    int32_t iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm1Pin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 1.");
        exit(EXIT_FAILURE);
    }
    PWMfrequency = iRealPWMfreq; // Now it is the Real PWM frequency
    if(set_PWM_range(gpioHostHandle, pwm1Pin, 255) < 0) {
        qDebug() << QString("Unable to set PWM range for Motor 1.");
        exit(EXIT_FAILURE);
    }
    int32_t realRange = get_PWM_real_range(gpioHostHandle, pwm1Pin);
    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }

    if(set_PWM_frequency(gpioHostHandle, pwm2Pin, PWMfrequency) < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 2.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_range(gpioHostHandle, pwm2Pin, realRange) < 0) {
        qDebug() << QString("Unable to set PWM range for Motor 2.");
        exit(EXIT_FAILURE);
    }
    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }

    //=======================================================================
    // The real range, the number of steps between fully off and fully
    // on for each of the 18 available GPIO frequencies is
    //=======================================================================
    //      25(#1),     50(#2),   100(#3),   125(#4),    200(#5),    250(#6),
    //     400(#7),    500(#8),   625(#9),  800(#10),  1000(#11),  1250(#12),
    //    2000(#13), 2500(#14), 4000(#15), 5000(#16), 10000(#17), 20000(#18).
    //=======================================================================
    // The real value set by set_PWM_range() is (dutycycle*real_range)/range.
    //=======================================================================

    initPins();
}


void
MotorController_L298::initPins() {

    // Motor 1 Pins
    if(set_mode(gpioHostHandle, mot1in1Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot1in1Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot1in1Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up") .arg(mot1in1Pin);
        exit(EXIT_FAILURE);
    }

    if(set_mode(gpioHostHandle, mot1in2Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot1in2Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot1in2Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot1in2Pin);
        exit(EXIT_FAILURE);
    }
    // Motor 2 Pins
    if(set_mode(gpioHostHandle, mot2in1Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot2in1Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot2in1Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot2in1Pin);
        exit(EXIT_FAILURE);
    }

    if(set_mode(gpioHostHandle, mot2in2Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot2in2Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot2in2Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot2in2Pin);
        exit(EXIT_FAILURE);
    }
}


MotorController_L298::~MotorController_L298() {
    if(gpioHostHandle>=0) {
        pigpio_stop(gpioHostHandle);
    }
}


int32_t
MotorController_L298::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void
MotorController_L298::move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
    if (rightSpeed < 0) {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0) {
        rightSpeed = max(rightSpeed, minAbsSpeed);
        rightSpeed = min(rightSpeed, 255);
    }
    
    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

    if (leftSpeed < 0) {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0) {
        leftSpeed = max(leftSpeed, minAbsSpeed);
        leftSpeed = min(leftSpeed, 255);
    }
    
    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

    gpio_write(gpioHostHandle, mot2in1Pin,
               rightSpeed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin,
               rightSpeed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot1in1Pin,
               leftSpeed  > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin,
               leftSpeed  > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, realRightSpeed * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, realLeftSpeed  * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }
}


void
MotorController_L298::move(int speed, int minAbsSpeed) {
    int direction = 1;
    
    if (speed < 0) {
        direction = -1;
        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }
    
    if (speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, abs(speed));
    
    gpio_write(gpioHostHandle, mot1in1Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in1Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, realSpeed * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, realSpeed  * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }
    currentSpeed = direction * realSpeed;
}


void
MotorController_L298::move(int speed) {
    if (speed == currentSpeed) return;
    
    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;
    
    gpio_write(gpioHostHandle, mot1in1Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in1Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, abs(speed) * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, abs(speed)  * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }
    currentSpeed = speed;
}


void
MotorController_L298::turnLeft(int speed, bool kick) {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_SET);
    
    if (kick) {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, 255) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, 255) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2.");
            exit(EXIT_FAILURE);
        }
        QThread::msleep(100);
    }
    
    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, speed * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, speed * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }
}


void
MotorController_L298::turnRight(int speed, bool kick) {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_SET);

    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_RESET);

    if (kick) {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, 255) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, 255) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2.");
            exit(EXIT_FAILURE);
        }
        QThread::msleep(100);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, speed * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, speed * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }
}


void
MotorController_L298::stopMoving() {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_RESET);

    if(set_PWM_dutycycle(gpioHostHandle, pwm1Pin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2Pin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2.");
        exit(EXIT_FAILURE);
    }

    currentSpeed = 0;
}
