#pragma once

#include <QTimer>

#include <QCoreApplication>
#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>
#include <QtNetwork/QAbstractSocket>
#include <QSettings>

#include "utilities.h"


QT_FORWARD_DECLARE_CLASS(QFile)
QT_FORWARD_DECLARE_CLASS(PID)
QT_FORWARD_DECLARE_CLASS(MotorController_L298)
QT_FORWARD_DECLARE_CLASS(MotorController_BST7960)
QT_FORWARD_DECLARE_CLASS(QTcpServer)
QT_FORWARD_DECLARE_CLASS(QTcpSocket)
QT_FORWARD_DECLARE_CLASS(QUdpSocket)


#define ACC_ADDR ADXL345_ADDR_ALT_LOW          // SDO connected to GND
//#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW

#define ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW  // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it

//#define L298
#define BST760


class MainWindow : public QCoreApplication
{
    Q_OBJECT

public:
    MainWindow(int &argc, char **argv);
    ~MainWindow();
    int  exec();

public slots:
    void onLoopTimeElapsed();
    void onStartAccCalibration();
    void onStartGyroCalibration();
    void onStartMagCalibration();
    void onStartStopPushed();
    void onNewTcpConnection();
    void onTcpClientDisconnected();
    void onTcpError(QAbstractSocket::SocketError error);
    void onReadFromServer();
    void onAHRSerror(QString sErrorString);

protected:
    void printMessage(QString sMessage);
    void initAHRSsensor();
    void initLayout();
    void restoreSettings();
    void saveSettings();
    bool isStationary();
    bool openTcpSession();
    void executeCommand(QString sMessage);
    void periodicUpdateWidgets();
    //    void readPendingDatagrams();

private:
    QSettings settings;
    ADXL345*  pAcc;
    ITG3200*  pGyro;
    HMC5883L* pMagn;
    Madgwick* pMadgwick;
    PID*      pPid;
#if defined(L298)
    MotorController_L298* pMotorController;
#elif defined(BST760)
    MotorController_BST7960* pMotorController;
#else
    #error "Undefined Motor Controller"
#endif

    QFile  *pLogFile;
    QTimer loopTimer;

    float ahrsSamplingFrequency;
    float values[9];
    float angles[3]; // yaw pitch roll
    float heading;

    int16_t temperature;
    long pressure;

    float GyroXOffset, GyroYOffset, GyroZOffset;

    uint64_t lastUpdate;
    uint64_t now;
    uint64_t t0;
    float deltaTime;

    float q0, q1, q2, q3;
    double angleX, angleY, angleZ;
    double avgX, avgY, avgZ;
    uint16_t nAvg, nCurr;

    bool bPIDControlInProgress;
    bool bAccCalInProgress;
    bool bGyroCalInProgress;
    bool bMagCalInProgress;

    QString       sMessage;
    QTcpServer*   pTcpServer;
    QTcpSocket*   pTcpServerConnection;
    int           serverPort;
    QByteArray    clientMessage;
    QUdpSocket*   pUdpSocket;
    int           udpPort;

    // PID
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    int moveState;
    int ControllerDirection;
    double movingAngleOffset;
    double input;
    double output;
    double motorSpeedFactorLeft;
    double motorSpeedFactorRight;
    volatile bool bCanContinue;
};
