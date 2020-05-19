#pragma once

#include <QWidget>
#include <QTimer>

#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>
#include <QtNetwork/QAbstractSocket>

#include "utilities.h"


QT_FORWARD_DECLARE_CLASS(QPushButton)
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


class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

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

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *event);
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
    ADXL345*  pAcc;
    ITG3200*  pGyro;
    HMC5883L* pMagn;
    Madgwick* pMadgwick;
    PID*      pPid;
    //MotorController_L298* pMotorController;
    MotorController_BST7960* pMotorController;

    QPushButton* buttonAccCalibration;
    QPushButton* buttonGyroCalibration;
    QPushButton* buttonMagCalibration;
    QPushButton* buttonStartStop;

    QTimer loopTimer;

    float samplingFrequency;
    float values[9];
    float angles[3]; // yaw pitch roll
    float heading;

    int16_t temperature;
    long pressure;

    float GyroXOffset, GyroYOffset, GyroZOffset;

    uint64_t lastUpdate;
    uint64_t now;
    uint64_t t0;
    float delta;
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
};
