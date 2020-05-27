//===============================================================
// To set the I2C speed @ 400KHz:
// sudo nano /boot/config.txt
// find the line with: dtparam=i2c_arm=on
// and change in     : dtparam=i2c_arm=on,i2c_arm_baudrate=400000
//===============================================================


//=======================
// Hardware Connections :
//=======================
//
// DFRobot 10DOF :
//      SDA on BCM 2:    pin 3 in the 40 pins GPIO connector
//      SCL on BCM 3:    pin 5 in the 40 pins GPIO connector
//      Vcc on 5V Power: pin 4 in the 40 pins GPIO connector
//      GND on GND:      pin 6 in the 40 pins GPIO connector
//
// For Raspberry Pi GPIO pin numbering see
// https://pinout.xyz/
//
// +5V on pins 2 or 4 in the 40 pin GPIO connector.
// GND on pins 6, 9, 14, 20, 25, 30, 34 or 39
// in the 40 pin GPIO connector.
//==============================================================

//==============================================================
// Information for connecting servos:
//
// Samwa servo pinout:
//      1) PWM Signal
//      2) GND
//      3) +5V
//==============================================================


// I2C Addresses: *************************************************
//      0x1E     HMC5883L_Address        (Magnetometer)
//      0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
//      0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
//      0x77     BMP085_ADDRESS          (Pressure and Temperature)
// ****************************************************************


// Info: ******************************************************
// The magnitude of the Earth's magnetic field at its surface
// ranges from 250 to 650 milli Gauss.
// ************************************************************


#include "mainwindow.h"
#include <cmath>

#include <QDebug>
#include <QFile>
#include <QStandardPaths>
#include <QThread>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QUdpSocket>

#include "PID_v1.h"
#if defined(L298)
    #include "MotorController_L298.h"
#elif defined(BST760)
    #include "MotorController_BST7960.h"
#else
    #error "Undefined Motor Controller"
#endif


#if defined(L298) // L298 MotorController
    #define PWM1_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
    #define M1IN1_PIN 17 // on BCM17: Pin 11 in the 40 pin GPIO connector.
    #define M1IN2_PIN 27 // on BCM27: Pin 13 in the 40 pin GPIO connector.
    #define PWM2_PIN  13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
    #define M2IN1_PIN 22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
    #define M2IN2_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.
#elif defined(BST760) // BST760 MotorController
    #define PWM1UP_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
    #define PWM1LOW_PIN 13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
    #define PWM2UP_PIN  22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
    #define PWM2LOW_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.
#else // No MotorController defined
    #error "Undefined Motor Controller"
#endif


#define MIN_ABS_SPEED        0

#define SAMPLING_FREQUENCY 100


//==============================================================
// Commands (Sent)              Meaning
//--------------------------------------------------------------
//      q               Quaternion Value
//      p               PID Values (time, input & output)
//      c               Robot Configuration Values
//      a               Accelerator  Raw Values
//      g               Gyroscope    Raw Values
//      m               Magnetometer Raw Values
//==============================================================

double times[100];
int indexTime = 0;


MainWindow::MainWindow(int &argc, char **argv)
    : QCoreApplication(argc, argv)
    , pAcc(nullptr)
    , pGyro(nullptr)
    , pMagn(nullptr)
    , pMadgwick(nullptr)
    , pPid(nullptr)
    , pMotorController(nullptr)
    , pLogFile(nullptr)
    // Status Variables
    , bPIDControlInProgress(false)
    , bAccCalInProgress(false)
    , bGyroCalInProgress(false)
    , bMagCalInProgress(false)
    // TCP-IP Server
    , pTcpServer(nullptr)
    , pTcpServerConnection(nullptr)
    , serverPort(43210)
    , pUdpSocket(nullptr)
    , udpPort(37755)
    // Motor Controller
    , motorSpeedFactorLeft(0.6)
    , motorSpeedFactorRight(0.6)
{
    // Open the log file
    QString sLogFileName = QString("10DOFlogfile.txt");
    QString sLogDir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    if(!sLogDir.endsWith(QString("/"))) sLogDir+= QString("/");
    sLogFileName = sLogDir+sLogFileName;
    pLogFile = new QFile(sLogFileName);
    if (!pLogFile->open(QIODevice::WriteOnly)) {
        qDebug() << QString("Unable to open file %1: %2.")
                    .arg(sLogFileName)
                    .arg(pLogFile->errorString());
        delete pLogFile;
        pLogFile = nullptr;
    }

    bCanContinue = true;
    setpoint = 0.0;
    ahrsSamplingFrequency = SAMPLING_FREQUENCY;

    restoreSettings();

    if(!openTcpSession())
        return;
    pUdpSocket = new QUdpSocket(this);

    pAcc  = new ADXL345(); // init ADXL345
    pGyro = new ITG3200(); // init ITG3200
    pMagn = new HMC5883L();// init HMC5883L

    connect(pAcc,  SIGNAL(error(QString)), this, SLOT(onAHRSerror(QString)));
    connect(pGyro, SIGNAL(error(QString)), this, SLOT(onAHRSerror(QString)));
    connect(pMagn, SIGNAL(error(QString)), this, SLOT(onAHRSerror(QString)));

    pMadgwick = new Madgwick();

#if defined(L298) // L298 MotorController
    pMotorController = new MotorController_L298(PWM1_PIN, M1IN1_PIN, M1IN2_PIN,
                                                PWM2_PIN, M2IN1_PIN, M2IN2_PIN,
                                                motorSpeedFactorLeft, motorSpeedFactorRight);
#elif defined(BST760) // BST760 MotorController
    pMotorController = new MotorController_BST7960(PWM1UP_PIN, PWM1LOW_PIN,
                                                   PWM2UP_PIN, PWM2LOW_PIN,
                                                   motorSpeedFactorLeft, motorSpeedFactorRight);
#endif
    pPid = new PID(Kp, Ki, Kd, ControllerDirection);

}


int
MainWindow::exec() {
    initAHRSsensor();
    if(!bCanContinue) return EXIT_FAILURE;
    pPid->SetSampleTime(100); // in ms
    pPid->SetOutputLimits(-255, 255);
    if(bPIDControlInProgress)
        pPid->SetMode(AUTOMATIC);
    else
        pPid->SetMode(MANUAL);

    pMadgwick->begin(ahrsSamplingFrequency);

    // Consider to change to QBasicTimer that it's faster than QTimer
    loopTimer.setTimerType(Qt::PreciseTimer);
    connect(&loopTimer, SIGNAL(timeout()),
            this, SLOT(onLoopTimeElapsed()));

    while(!pMagn->isDataReady()) {} // Wait for the slower first...
    pMagn->ReadScaledAxis(&values[6]);
    while(!pAcc->getInterruptSource(7)) {}
    pAcc->get_Gxyz(&values[0]);
    while(!pGyro->isRawDataReadyOn()) {}
    pGyro->readGyro(&values[3]);

    // Simulate a 180s of Readings to get the Starting Pose
    for(int i=0; i<180*ahrsSamplingFrequency; i++) {
        pMadgwick->update(values[3], values[4], values[5],
                          values[0], values[1], values[2],
                          values[6], values[7], values[8]);
    }

    lastUpdate = micros();
    now = lastUpdate;
    loopTimer.start(int32_t(1000.0/ahrsSamplingFrequency+0.5));
    sMessage = QString("Ready to be connected");
    printMessage(sMessage);
    return QCoreApplication::exec();
}


MainWindow::~MainWindow() {
    pMotorController->stopMoving();
    loopTimer.stop();

    saveSettings();

    if(pTcpServer) {
        pTcpServer->close();
        delete pTcpServer;
    }
    if(pUdpSocket) {
        pUdpSocket->close();
        delete pUdpSocket;
        pUdpSocket = nullptr;
    }
    if(pPid)             delete pPid;
    if(pMadgwick)        delete pMadgwick;
    if(pMotorController) delete pMotorController;
    if(pMagn)            delete pMagn;
    if(pGyro)            delete pGyro;
    if(pAcc)             delete pAcc;
    if(pLogFile) {
        if(pLogFile->isOpen()) {
            pLogFile->flush();
        }
        pLogFile->deleteLater();
    }
}


void
MainWindow::printMessage(QString sMessage) {
    if(pLogFile) {
        if(pLogFile->isOpen()) {
            pLogFile->write(sMessage.toUtf8().data());
            pLogFile->write("\n");
            pLogFile->flush();
        }
        else
            qDebug() << sMessage;
    }
    else
        qDebug() << sMessage;
}


void
MainWindow::onAHRSerror(QString sErrorString) {
    printMessage(sErrorString);
    bCanContinue = false;
}


void
MainWindow::restoreSettings() {
    // Gyroscope
    GyroXOffset = settings.value("GyroXOffset", 1.0).toFloat();
    GyroYOffset = settings.value("GyroYOffset", 1.0).toFloat();
    GyroZOffset = settings.value("GyroZOffset", 1.0).toFloat();
    // PID
    ControllerDirection = settings.value("ControllerDirection", 0).toInt();
    Kp                  = settings.value("Kp",                  1.0).toDouble();
    Kd                  = settings.value("Kd",                  0.0).toDouble();
    Ki                  = settings.value("Ki",                  0.0).toDouble();
    setpoint            = settings.value("setPoint",            0.0).toDouble();
    // Motor Controller
    motorSpeedFactorLeft = settings.value("motorSpeedFactorLeft",  0.6).toDouble();
    motorSpeedFactorRight= settings.value("motorSpeedFactorRight", 0.6).toDouble();
}


void
MainWindow::saveSettings() {
    // Gyroscope
    settings.setValue("GyroXOffset", pGyro->offsets[0]);
    settings.setValue("GyroYOffset", pGyro->offsets[1]);
    settings.setValue("GyroZOffset", pGyro->offsets[2]);
    // PID
    settings.setValue("ControllerDirection", ControllerDirection);
    settings.setValue("Kp", pPid->GetKp());
    settings.setValue("Kd", pPid->GetKd());
    settings.setValue("Ki", pPid->GetKi());
    settings.setValue("setPoint", setpoint);
    // Motor Controller
    settings.setValue("motorSpeedFactorLeft",  motorSpeedFactorLeft);
    settings.setValue("motorSpeedFactorRight", motorSpeedFactorRight);
}


bool
MainWindow::openTcpSession() {
    pTcpServer = new QTcpServer(this);
    if(!pTcpServer->listen(QHostAddress::Any, serverPort)) {
        QString sMessage = QString("TCP-IP Unable to start listen()");
        printMessage(sMessage);
    }
    connect(pTcpServer, SIGNAL(newConnection()),
            this, SLOT(onNewTcpConnection()));
    connect(pTcpServer, SIGNAL(acceptError(QAbstractSocket::SocketError)),
            this, SLOT(onTcpError(QAbstractSocket::SocketError)));
    QString ipAddress;
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
    // use the first non-localhost IPv4 address
    for(qint32 i=0; i<ipAddressesList.size(); ++i) {
        if(ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address()) {
            ipAddress = ipAddressesList.at(i).toString();
            if(ipAddress.left(3) != QString("169"))
                break;
        }
    }
    // if we did not find one, use IPv4 localhost
    if(ipAddress.isEmpty())
        ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
    QString sMessage = QString("Running TCP-IP server at address %1 port:%2")
                       .arg(ipAddress)
                       .arg(pTcpServer->serverPort());
    printMessage(sMessage);
    return true;
}


void
MainWindow::onTcpError(QAbstractSocket::SocketError error) {
    QString sMessage;
    if(error == QAbstractSocket::ConnectionRefusedError)
        sMessage = QString("The connection was refused by the peer (or timed out).");
    else if(error == QAbstractSocket::RemoteHostClosedError) {
        sMessage = QString("The remote host closed the connection.");
    } else if(error == QAbstractSocket::HostNotFoundError)
        sMessage = QString("The host address was not found.");
    else if(error == QAbstractSocket::SocketAccessError)
        sMessage = QString("The socket operation failed because the application lacked the required privileges.");
    else if(error == QAbstractSocket::SocketResourceError)
        sMessage = QString("The local system ran out of resources (e.g., too many sockets).");
    else if(error == QAbstractSocket::SocketTimeoutError)
        sMessage = QString("The socket operation timed out.");
    else if(error == QAbstractSocket::DatagramTooLargeError)
        sMessage = QString("The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).");
    else if(error == QAbstractSocket::NetworkError)
        sMessage = QString("An error occurred with the network (e.g., the network cable was accidentally plugged out).");
    else if(error == QAbstractSocket::AddressInUseError)
        sMessage = QString("The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.");
    else if(error == QAbstractSocket::SocketAddressNotAvailableError)
        sMessage = QString("The address specified to QAbstractSocket::bind() does not belong to the host.");
    else if(error == QAbstractSocket::UnsupportedSocketOperationError)
        sMessage = QString("The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).");
    else if(error == QAbstractSocket::ProxyAuthenticationRequiredError)
        sMessage = QString("The socket is using a proxy, and the proxy requires authentication.");
    else if(error == QAbstractSocket::SslHandshakeFailedError)
        sMessage = QString("The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)");
    else if(error == QAbstractSocket::UnfinishedSocketOperationError)
        sMessage = QString("Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).");
    else if(error == QAbstractSocket::ProxyConnectionRefusedError)
        sMessage = QString("Could not contact the proxy server because the connection to that server was denied");
    else if(error == QAbstractSocket::ProxyConnectionClosedError)
        sMessage = QString("The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)");
    else if(error == QAbstractSocket::ProxyConnectionTimeoutError)
        sMessage = QString("The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.");
    else if(error == QAbstractSocket::ProxyNotFoundError)
        sMessage = QString("The proxy address set with setProxy() (or the application proxy) was not found.");
    else if(error == QAbstractSocket::ProxyProtocolError)
        sMessage = QString("The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.");
    else if(error == QAbstractSocket::OperationError)
        sMessage = QString("An operation was attempted while the socket was in a state that did not permit it.");
    else if(error == QAbstractSocket::SslInternalError)
        sMessage = QString("The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.");
    else if(error == QAbstractSocket::SslInvalidUserDataError)
        sMessage = QString("Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.");
    else if(error == QAbstractSocket::TemporaryError)
        sMessage = QString("A temporary error occurred (e.g., operation would block and socket is non-blocking).");
    else if(error == QAbstractSocket::UnknownSocketError)
        sMessage = QString("An unidentified error occurred.");
    printMessage(sMessage);
}


void
MainWindow::onNewTcpConnection() {
    pTcpServerConnection = pTcpServer->nextPendingConnection();
    if(pTcpServerConnection->isValid()) {
        connect(pTcpServerConnection, SIGNAL(readyRead()),
                this, SLOT(onReadFromServer()));
        connect(pTcpServerConnection, SIGNAL(error(QAbstractSocket::SocketError)),
                this, SLOT(onTcpError(QAbstractSocket::SocketError)));
        connect(pTcpServerConnection, SIGNAL(disconnected()),
                this, SLOT(onTcpClientDisconnected()));
        sMessage = QString("Connected to: %1")
                    .arg(pTcpServerConnection->peerAddress().toString());
        printMessage(sMessage);

        t0 = micros();
    }
}


void
MainWindow::onTcpClientDisconnected() {
    QString sClient = pTcpServerConnection->peerAddress().toString();
    pTcpServerConnection = nullptr;
    sMessage = QString("Disconnected from: %1").arg(sClient);
    printMessage(sMessage);
    loopTimer.stop();
    pGyro->zeroCalibrate(600);
    saveSettings();
    loopTimer.start(int32_t(1000.0/ahrsSamplingFrequency+0.5));
}


void
MainWindow::onReadFromServer() {
    clientMessage.append(pTcpServerConnection->readAll());
    QString sMessage = QString(clientMessage);
    int32_t iPos = sMessage.indexOf('#');
    while(iPos > 0) {
        QString sCommand = sMessage.left(iPos);
        executeCommand(sCommand);
        sMessage = sMessage.mid(iPos+1);
        iPos = sMessage.indexOf('#');
    }
    clientMessage = sMessage.toLatin1();
}


//==============================================================
// Orders (Received)            Meaning
//--------------------------------------------------------------
//      G           Set PID Control
//      S           Set Manual Control
//      P           Send PID Values (Kp, Ki, Kd)
//      C           Ask Robot Configuration
//      M           Manual Move at a given speed Left & Rigth
//      K           Switch off the Program
//==============================================================
void
MainWindow::executeCommand(QString sMessage) {
    QStringList tokens = sMessage.split(' ');
    if(tokens.at(0) == QString("G")) { // Set PID Control On
        bPIDControlInProgress = true;
        pPid->SetMode(AUTOMATIC);
    }
    else if(tokens.at(0) == QString("S")) { // Set PID Control Off
        bPIDControlInProgress = false;
        pPid->SetMode(MANUAL);
    }
    else if(tokens.at(0) == QString("C")) { // Send Robot Configuration
        if(pTcpServerConnection)
            if(pTcpServerConnection->isValid()) {
                sMessage = QString("c %1 %2 %3 %4 %5 %6#")
                         .arg(pPid->GetKp())
                         .arg(pPid->GetKi())
                         .arg(pPid->GetKd())
                         .arg(motorSpeedFactorLeft)
                         .arg(motorSpeedFactorRight)
                         .arg(setpoint);
                pTcpServerConnection->write(sMessage.toLatin1());
            }
    }
    else if(tokens.at(0) == QString("M")) { // Start Manual Move
        if(tokens.length() < 3) return;
        double vLeft  = tokens.at(1).toDouble();
        double vRight = tokens.at(2).toDouble();
        pMotorController->move(vLeft, vRight, 0);
    }
    else if(tokens.at(0) == QString("P")) {// Set PID Parameters
        if(tokens.length() < 5) return;
        double kp = tokens.at(1).toDouble();
        double ki = tokens.at(2).toDouble();
        double kd = tokens.at(3).toDouble();
        setpoint = tokens.at(4).toDouble();
        pPid->SetTunings(kp, ki, kd);
    }
    else if(tokens.at(0) == QString("K")) { // Close Program
        pMotorController->stopMoving();
        loopTimer.stop();
        saveSettings();
        if(pTcpServer) {
            pTcpServer->close();
            delete pTcpServer;
            pTcpServer = nullptr;
        }
        if(pUdpSocket) {
            pUdpSocket->close();
            delete pUdpSocket;
            pUdpSocket = nullptr;
        }
        sMessage = QString("Program Regularly Switched Off. Bye");
        printMessage(sMessage);
        if(pLogFile) {
            if(pLogFile->isOpen()) {
                pLogFile->flush();
            }
            pLogFile->deleteLater();
            pLogFile = nullptr;
        }
        exit(EXIT_SUCCESS);
    }
    //qDebug() << "Received: " << sMessage;
}


bool
MainWindow::isStationary() {
    uint16_t nSamples = 600;
    QVector<double> accVal;
    double avg[3] = {0};
    for(uint16_t i=0; i<nSamples; i++) {
        if(pAcc->getInterruptSource(7)) { // Accelerator Data Ready
            pAcc->get_Gxyz(&values[0]);
        }
        avg[0] += values[0];
        avg[1] += values[1];
        avg[2] += values[2];
        accVal.append(sqrt(values[0]*values[0] +
                           values[1]*values[1] +
                           values[2]*values[2])
                     );
    }
    avg[0] /= double(nSamples);
    avg[1] /= double(nSamples);
    avg[2] /= double(nSamples);
    double average = sqrt((avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]));
//    qDebug() << "Acc Avg=" << average;
    double sigma = 0;
    for(uint16_t i=0; i<nSamples; i++) {
        sigma += pow((accVal.at(i)-average), 2.0);
    }
    sigma /= double(nSamples);
//    qDebug() << "Acc Sgm=" << sqrt(sigma);
    // Noise Level (from datasheet = 1.1 LSB (3.9mg/LSB) )
    return sigma < 3.0*0.0039;
}


void
MainWindow::initAHRSsensor() {
    if(!pAcc->init(ACC_ADDR))
        return;
    pAcc->setRangeSetting(2); // +/-2g. Possible values are: 2g, 4g, 8g, 16g

    if(!pGyro->init(ITG3200_DEF_ADDR)) return;
    if(isStationary()) { // Gyro calibration done only when stationary
        pGyro->zeroCalibrate(600); // calibrate the ITG3200
        sMessage = QString("Gyro Offsets from calibration: %1 %2 %3")
                .arg(pGyro->offsets[0])
                .arg(pGyro->offsets[1])
                .arg(pGyro->offsets[2]);
        printMessage(sMessage);
    }
    else {
        pGyro->setOffsets(GyroXOffset, GyroYOffset, GyroZOffset);
        sMessage = QString("Gyro Offsets from saved values: %1 %2 %3")
                .arg(pGyro->offsets[0])
                .arg(pGyro->offsets[1])
                .arg(pGyro->offsets[2]);
        printMessage(sMessage);
    }

    if(!pMagn->init()) return;
    pMagn->SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    pMagn->SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

    //bmp085Calibration(); // init barometric pressure sensor
}


void
MainWindow::onStartStopPushed() {
//    if(bRunInProgress) {
//        pMotorController->stopMoving();
//        loopTimer.stop();
//        bRunInProgress = false;
//        if(bAccCalInProgress)
//            onStartAccCalibration();
//        if(bGyroCalInProgress)
//            onStartGyroCalibration();
//        if(bMagCalInProgress)
//            onStartMagCalibration();
//    }
//    else {
//        bRunInProgress = true;
//        lastUpdate = micros();
//        now = lastUpdate;
//        loopTimer.start(int32_t(1000.0/ahrsSamplingFrequency+0.5));
//    }
}


void
MainWindow::onStartAccCalibration() {
    if(bAccCalInProgress) {
        bAccCalInProgress = false;
    }
    else {
        bGyroCalInProgress = false;
        bMagCalInProgress = false;
        bAccCalInProgress = true;

        avgX = avgY = avgZ = 0.0;
        nAvg = 10;
        nCurr = 0;
        t0 = micros();
    }
}


void
MainWindow::onStartGyroCalibration() {
    if(bGyroCalInProgress) {
        bGyroCalInProgress = false;
    }
    else {
        bMagCalInProgress = false;
        bAccCalInProgress = false;
        bGyroCalInProgress = true;

        angleX = 0.0;
        angleY = 0.0;
        angleZ = 0.0;
        t0 = micros();
    }
}


void
MainWindow::onStartMagCalibration() {
    if(bMagCalInProgress) {
        bMagCalInProgress = false;
    }
    else {
        bAccCalInProgress = false;
        bGyroCalInProgress = false;
        bMagCalInProgress = true;

        t0 = micros();
    }
}


void
MainWindow::onLoopTimeElapsed() {
    //==================================================================
    //  !!! Attention !!!
    //==================================================================
    // Reasonable convergence can be achieved in two or three iterations
    // meaning that we should operate this sensor fusion filter at a
    // rate two or three times the output data rate of the sensor.
    //
    // Deliver sensor values at the Madgwick algorithm
    // in the expected format which is degrees/s, m/s² and mG (milliGauss)
    //==================================================================

    now        = micros();
    deltaTime  = float(now-lastUpdate)/1000000.f;
    lastUpdate = now;
    pMadgwick->setInvFreq(deltaTime);

    if(pMagn->isDataReady()) { // The Slower Sensor First
        pMagn->ReadScaledAxis(&values[6]);
    }

    if(pAcc->getInterruptSource(7)) { // Accelerator Data Ready
        pAcc->get_Gxyz(&values[0]);
    }

    if(pGyro->isRawDataReadyOn()) { // Gyroscope Data Ready
        pGyro->readGyro(&values[3]);
    }

    pMadgwick->update(values[3], values[4], values[5],
                      values[0], values[1], values[2],
                      values[6], values[7], values[8]);

    input  = pMadgwick->getPitch();
    output = pPid->Compute(input, setpoint);
    pMotorController->move(output, MIN_ABS_SPEED);
    periodicUpdateWidgets();

}


void
MainWindow::periodicUpdateWidgets() {
    if(pTcpServerConnection) {
        if(pTcpServerConnection->isValid()) {
            pMadgwick->getRotation(&q0, &q1, &q2, &q3);
            sMessage = QString("q %1 %2 %3 %4#")
                       .arg(q0)
                       .arg(q1)
                       .arg(q2)
                       .arg(q3);
            double x = double(now-t0)/1000000.0;
            if(bPIDControlInProgress) {
                sMessage += QString("p %1 %2 %3#")
                            .arg(x)
                            .arg(double(input-setpoint))
                            .arg(double(output/Kp));
            }
            else {
                sMessage += QString("p %1 %2#")
                            .arg(x)
                            .arg(double(input-setpoint));
            }
            pUdpSocket->writeDatagram(sMessage.toLatin1(),
                                      pTcpServerConnection->peerAddress(),
                                      udpPort);
        }
    }
}

