QT += core
QT += gui
QT += widgets
Qt += multimedia

TARGET = 10Dof_NoGUI
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS


SOURCES += main.cpp
SOURCES += mainwindow.cpp
SOURCES += HMC5883L.cpp
SOURCES += MadgwickAHRS.cpp
SOURCES += MahonyAHRS.cpp
SOURCES += ADXL345.cpp
SOURCES += ITG3200.cpp
SOURCES += PID_v1.cpp
SOURCES += utilities.cpp
SOURCES += MotorController_BST7960.cpp
SOURCES += MotorController_L298.cpp

HEADERS += mainwindow.h
HEADERS += HMC5883L.h
HEADERS += MadgwickAHRS.h
HEADERS += MahonyAHRS.h
HEADERS += ADXL345.h
HEADERS += ITG3200.h
HEADERS += PID_v1.h
HEADERS += MotorController_BST7960.h
HEADERS += MotorController_L298.h
HEADERS += utilities.h

LIBS += -lpigpiod_if2 # To include libpigpiod_if2.so from /usr/local/lib
LIBS += -lQt5Network

FORMS +=

RESOURCES +=

DISTFILES += Inertial-Frame.png \
DISTFILES += .gitignore
