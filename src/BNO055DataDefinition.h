
/*
 * Copyright 2019 <Lennart Nachtigall> <lennart.nachtigall@firesurfer.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef BNO055DATADEFINITION_H
#define BNO055DATADEFINITION_H

#include <stdint.h>
struct BNO055_Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

struct BNO055_Gyro
{
    double x;
    double y;
    double z;
};

struct BNO055_Euler
{
    double heading;
    double roll;
    double pitch;
};
struct BNO055_LinearAcceleration
{
    double x;
    double y;
    double z;
};

struct BNO055Revision
{
    int accelerometer;
    int magnetometer;
    int gyro;
    int bootloader;
    int software;
};

struct BNO055SelftestResult
{
    uint8_t selftest;
    uint8_t error;
    uint8_t status;
};

struct BNO055CalibrationStatus
{
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
};

#endif // BNO055DATADEFINITION_H
