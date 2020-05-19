#include "MotorController_BST7960.h"
#include "pigpiod_if2.h" // The library to use GPIO pins on Raspberry


#include <algorithm> // min() & max()
#include "string.h"  // for memset()
#include <QDebug>
#include <QThread>

using namespace std; // min() & max()


#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1


template
<typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//=================================================================================
//    43A High Power BTS7960 DC Motor Driver Module. http://handsontec.com
//=================================================================================
//     Pin No    Function                      Description
//---------------------------------------------------------------------------------
//        1         RPWM        Forward Level or PWM signal, Active High
//        2         LPWM        Reverse Level or PWM signal, Active High
//        3         R_EN        Forward Drive Enable Input, Active High
//        4         L_EN        Reverse Drive Enable Input, Active High
//        5         R_IS        Forward Drive, Side current alarm output (Not used)
//        6         L_IS        Reverse Drive, Side current alarm output (Not used)
//        7         Vcc         +5V Power Supply Input to the Logic
//        8         Gnd         Ground: Connect to microcontroller GND
//=================================================================================


MotorController_BST7960::MotorController_BST7960(uint32_t pwm1Up, uint32_t pwm1Low,
                                                 uint32_t pwm2Up, uint32_t pwm2Low,
                                                 double _motor1Const, double _motor2Const)
{
    pwm1UpPin  = pwm1Up;
    pwm1LowPin = pwm1Low;

    pwm2UpPin  = pwm2Up;
    pwm2LowPin = pwm2Low;

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

    // WARNING !!! a too low frequency will interfere with the IMU readings !!!
    PWMfrequency = 8000; // in Hz

    gpioHostHandle = pigpio_start((char*)"localhost", (char*)"8888");
    if(gpioHostHandle < 0) {
        qDebug() << QString("Unable to initialize GPIO");
        exit(EXIT_FAILURE);
    }

// Motor 1
    // set_PWM_frequency() returns the numerically closest frequency if OK
    int32_t iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm1UpPin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 1Up.");
        exit(EXIT_FAILURE);
    }
    PWMfrequency = iRealPWMfreq; // Now it is the Real PWM frequency
    iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm1LowPin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 1Low.");
        exit(EXIT_FAILURE);
    }

    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Low.");
        exit(EXIT_FAILURE);
    }

// Motor 2
    // set_PWM_frequency() returns the numerically closest frequency if OK
    iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm2UpPin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 2Up.");
        exit(EXIT_FAILURE);
    }
    PWMfrequency = iRealPWMfreq; // Now it is the Real PWM frequency
    iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm2LowPin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Unable to set the PWM frequency for Motor 2Low.");
        exit(EXIT_FAILURE);
    }

    //Start (non-zero dutycycle) or stop (0) PWM pulses on the GPIO.
    if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Low.");
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

    currentSpeed = 0;
}


MotorController_BST7960::~MotorController_BST7960() {
    if(gpioHostHandle >= 0) {
        pigpio_stop(gpioHostHandle);
    }
}


int32_t
MotorController_BST7960::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void
MotorController_BST7960::move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
    int absSpeed = min(max(abs(rightSpeed), minAbsSpeed), 255);
    int realRightSpeed = sgn(rightSpeed)*map(absSpeed, 0, 255, minAbsSpeed, 255);

    absSpeed = min(max(abs(leftSpeed), minAbsSpeed), 255);
    int realLeftSpeed = sgn(leftSpeed)*map(abs(absSpeed), 0, 255, minAbsSpeed, 255);

// Motor 1
    if(realRightSpeed > 0) {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, abs(realRightSpeed) * motor1Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Up.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Low.");
            exit(EXIT_FAILURE);
        }
    }
    else {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, abs(realRightSpeed) * motor1Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Low.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Up.");
            exit(EXIT_FAILURE);
        }
    }

// Motor 2
    if(realLeftSpeed > 0) {
        if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, abs(realLeftSpeed) * motor2Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Up.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Low.");
            exit(EXIT_FAILURE);
        }
    }
    else {
        if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, abs(realLeftSpeed) * motor2Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Low.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Up.");
            exit(EXIT_FAILURE);
        }
    }

}


void
MotorController_BST7960::move(int speed, int minAbsSpeed) {
    int direction = sgn(speed);
    speed = min(max(abs(speed), minAbsSpeed), 255);

    if(direction*speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, speed);

    if(direction > 0) {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, abs(realSpeed) * motor1Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Up.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Low.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, abs(realSpeed) * motor2Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Up.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Low.");
            exit(EXIT_FAILURE);
        }
    }
    else {
        if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, abs(realSpeed) * motor1Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Low.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 1Up.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, abs(realSpeed) * motor2Const) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Low.");
            exit(EXIT_FAILURE);
        }
        if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, 0) < 0) {
            qDebug() << QString("Unable to start the PWM for Motor 2Up.");
            exit(EXIT_FAILURE);
        }
    }

    currentSpeed = direction * realSpeed;
}


void
MotorController_BST7960::move(int speed) {
    if (speed == currentSpeed) return;
    speed = min(max(abs(speed), 0), 255);

    if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, speed * motor1Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Low.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, speed * motor2Const) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Low.");
        exit(EXIT_FAILURE);
    }

    currentSpeed = speed;
}


void
MotorController_BST7960::turnLeft(int speed, bool kick) {
    Q_UNUSED(speed)
// ToDo:
    if (kick) {
        QThread::msleep(100);
    }
}


void
MotorController_BST7960::turnRight(int speed, bool kick) {
    Q_UNUSED(speed)
// ToDo:
    if (kick) {
        QThread::msleep(100);
    }
}


void
MotorController_BST7960::stopMoving() {
    if(set_PWM_dutycycle(gpioHostHandle, pwm1UpPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm1LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 1Low.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2UpPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Up.");
        exit(EXIT_FAILURE);
    }
    if(set_PWM_dutycycle(gpioHostHandle, pwm2LowPin, 0) < 0) {
        qDebug() << QString("Unable to start the PWM for Motor 2Low.");
        exit(EXIT_FAILURE);
    }

    currentSpeed = 0;
}
