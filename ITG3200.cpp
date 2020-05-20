/****************************************************************************
* ITG3200.cpp - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/

#include "ITG3200.h"
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <QDebug>
#include <QThread>


using namespace std;


ITG3200::ITG3200()
    : QObject()
{
    setGains(1.0, 1.0, 1.0);
    setOffsets(0.0, 0.0, 0.0);
    setRevPolarity(false, false, false);
}


bool
ITG3200::init(uint16_t  address) {
    // Uncomment or change your default ITG3200 initialization

    // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
    return init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);

    // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
    //init(address, NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);

    // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
    //init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);

    // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
    //init(address, NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);
}


bool
ITG3200::init(uint16_t address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
    _dev_address = address;
    // open device on /dev/i2c-1
    if((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        emit error(QString("ITG3200 Error: Couldn't open device %1 (%2)").arg(__FILE__).arg(__LINE__));
        return false;
    }
    // start transmission to device
    if(ioctl(fd, I2C_SLAVE, _dev_address) == -1) {
        emit error(QString("ITG3200 Errorin ioctl() %1 (%2)").arg(__FILE__).arg(__LINE__));
        return false;
    }
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setSampleRateDiv(_SRateDiv);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setFSRange(_Range);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setFilterBW(_filterBW);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setClockSource(_ClockSrc);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setITGReady(_ITGReady);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    setRawDataReady(_INTRawDataReady);
    QThread::msleep(GYROSTART_UP_DELAY);  // startup
    return true;
}


byte
ITG3200::getDevAddr() {
  /*readmem(WHO_AM_I, 1, &_buff[0]);
    return _buff[0];  */
    return _dev_address;
}


void
ITG3200::setDevAddr(uint16_t  _addr) {
    writemem(WHO_AM_I, _addr);
    _dev_address = _addr;
}


byte
ITG3200::getSampleRateDiv() {
    readmem(SMPLRT_DIV, 1, &_buff[0]);
    return _buff[0];
}


void
ITG3200::setSampleRateDiv(byte _SampleRate) {
    writemem(SMPLRT_DIV, _SampleRate);
}


byte
ITG3200::getFSRange() {
    readmem(DLPF_FS, 1, &_buff[0]);
    return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}


void
ITG3200::setFSRange(byte _Range) {
    readmem(DLPF_FS, 1, &_buff[0]);
    writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) );
}


byte
ITG3200::getFilterBW() {
    readmem(DLPF_FS, 1, &_buff[0]);
    return (_buff[0] & DLPFFS_DLPF_CFG);
}


void
ITG3200::setFilterBW(byte _BW) {
    readmem(DLPF_FS, 1, &_buff[0]);
    writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW));
}


bool
ITG3200::isINTActiveOnLow() {
    readmem(INT_CFG, 1, &_buff[0]);
    return ((_buff[0] & INTCFG_ACTL) >> 7);
}


void
ITG3200::setINTLogiclvl(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7)));
}


bool
ITG3200::isINTOpenDrain() {
    readmem(INT_CFG, 1, &_buff[0]);
    return ((_buff[0] & INTCFG_OPEN) >> 6);
}


void
ITG3200::setINTDriveType(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6));
}


bool
ITG3200::isLatchUntilCleared() {
    readmem(INT_CFG, 1, &_buff[0]);
    return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}


void
ITG3200::setLatchMode(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5));
}


bool
ITG3200::isAnyRegClrMode() {
    readmem(INT_CFG, 1, &_buff[0]);
    return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}


void
ITG3200::setLatchClearMode(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4));
}


bool
ITG3200::isITGReadyOn() {
    readmem(INT_CFG, 1, &_buff[0]);
    return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}


void
ITG3200::setITGReady(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2));
}


bool
ITG3200::isRawDataReadyOn() {
    readmem(INT_CFG, 1, &_buff[0]);
    return (_buff[0] & INTCFG_RAW_RDY_EN);
}


void
ITG3200::setRawDataReady(bool _State) {
    readmem(INT_CFG, 1, &_buff[0]);
    writemem(INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State));
}


bool
ITG3200::isITGReady() {
    readmem(INT_STATUS, 1, &_buff[0]);
    return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}


bool
ITG3200::isRawDataReady() {
    readmem(INT_STATUS, 1, &_buff[0]);
    return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}


void
ITG3200::readTemp(float *_Temp) {
    readmem(TEMP_OUT, 2, _buff);
    *_Temp = 35 + (((_buff[0] << 8) | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32
}


void
ITG3200::readGyroRaw(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ){
    readmem(GYRO_XOUT, 6, _buff);
    *_GyroX = ((_buff[0] << 8) | _buff[1]);
    *_GyroY = ((_buff[2] << 8) | _buff[3]);
    *_GyroZ = ((_buff[4] << 8) | _buff[5]);
}


void
ITG3200::readGyroRaw(int16_t *_GyroXYZ){
    readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
    polarities[0] = _Xpol ? -1 : 1;
    polarities[1] = _Ypol ? -1 : 1;
    polarities[2] = _Zpol ? -1 : 1;
}


void
ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain) {
    gains[0] = _Xgain;
    gains[1] = _Ygain;
    gains[2] = _Zgain;
}


void
ITG3200::setOffsets(int16_t _Xoffset, int16_t _Yoffset, int16_t _Zoffset) {
    offsets[0] = _Xoffset;
    offsets[1] = _Yoffset;
    offsets[2] = _Zoffset;
}


void
ITG3200::zeroCalibrate(uint16_t totSamples, uint16_t sampleDelayMS) {
    int16_t xyz[3];
    float tmpOffsets[] = {0.0f, 0.0f, 0.0f};

    for(uint16_t i=0; i<totSamples; i++) {
        QThread::msleep(sampleDelayMS);
        readGyroRaw(xyz);
        tmpOffsets[0] += xyz[0];
        tmpOffsets[1] += xyz[1];
        tmpOffsets[2] += xyz[2];
    }
    setOffsets(-int16_t(tmpOffsets[0]/totSamples),
               -int16_t(tmpOffsets[1]/totSamples),
               -int16_t(tmpOffsets[2]/totSamples));
}


void
ITG3200::readGyroRawCal(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ) {
    readGyroRaw(_GyroX, _GyroY, _GyroZ);
    *_GyroX += offsets[0];
    *_GyroY += offsets[1];
    *_GyroZ += offsets[2];
}


void
ITG3200::readGyroRawCal(int16_t *_GyroXYZ) {
    readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ){
    int16_t x, y, z;
     // x,y,z will contain calibrated integer values from the sensor
    readGyroRawCal(&x, &y, &z);
    // sensitivity is 14.375 LSBs per °/sec
    *_GyroX =  x / 14.375 * polarities[0] * gains[0];
    *_GyroY =  y / 14.375 * polarities[1] * gains[1];
    *_GyroZ =  z / 14.375 * polarities[2] * gains[2];
}


void
ITG3200::readGyro(float *_GyroXYZ){
    readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::reset() {
    writemem(PWR_MGM, PWRMGM_HRESET);
    QThread::msleep(GYROSTART_UP_DELAY); //gyro startup
}


bool
ITG3200::isLowPower() {
    readmem(PWR_MGM, 1, &_buff[0]);
    return (_buff[0] & PWRMGM_SLEEP) >> 6;
}


void
ITG3200::setPowerMode(bool _State) {
    readmem(PWR_MGM, 1, &_buff[0]);
    writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));
}


bool
ITG3200::isXgyroStandby() {
    readmem(PWR_MGM, 1, &_buff[0]);
    return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}


bool
ITG3200::isYgyroStandby() {
    readmem(PWR_MGM, 1, &_buff[0]);
    return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}


bool
ITG3200::isZgyroStandby() {
    readmem(PWR_MGM, 1, &_buff[0]);
    return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}


void
ITG3200::setXgyroStandby(bool _Status) {
    readmem(PWR_MGM, 1, &_buff[0]);
    writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}


void
ITG3200::setYgyroStandby(bool _Status) {
    readmem(PWR_MGM, 1, &_buff[0]);
    writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}


void
ITG3200::setZgyroStandby(bool _Status) {
    readmem(PWR_MGM, 1, &_buff[0]);
    writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}


byte
ITG3200::getClockSource() {
    readmem(PWR_MGM, 1, &_buff[0]);
    return (_buff[0] & PWRMGM_CLK_SEL);
}


void
ITG3200::setClockSource(byte _CLKsource) {
    readmem(PWR_MGM, 1, &_buff[0]);
    writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource));
}


void
ITG3200::writemem(uint8_t _addr, uint8_t _val) {
    std::array<uint8_t, 2> data{_addr, _val};
    if(write(fd, data.data(), data.size()) == -1) {
        emit error(QString("ITG3200 Error: write() %1 (%2)").arg(__FILE__).arg(__LINE__));
        return;
    }
}


void
ITG3200::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
    // sends register address to read from
    if(write(fd, &_addr, sizeof(_addr)) == -1) {
        emit error(QString("ITG3200 Error: write() %1 (%2)").arg(__FILE__).arg(__LINE__));
        return;
    }
    // receive DATA
    if(read(fd, __buff, _nbytes) != _nbytes) {
        emit error(QString("HMC5883L Error: read() %1 (%2)").arg(__FILE__).arg(__LINE__));
        return;
    }
}
