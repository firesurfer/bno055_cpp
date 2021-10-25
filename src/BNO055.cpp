
/*
 * Copyright 2021 <Lennart Nachtigall> <lennart.nachtigall@firesurfer.de>
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

#include "BNO055.h"

#include <cstring>
#include <thread>
#include <chrono>
#include <iostream>

BNO055::BNO055(     
        fn_flush_t _flush,
        fn_bytes_available_t _available,
        fn_write_bytes_t _write,
        fn_read_bytes_t _read,
        bool _debug):
    ser_bytes_available(_available),
    ser_read_bytes(_read),
    ser_write_bytes(_write),
    ser_flush(_flush),
    debug(_debug)
{

}

bool BNO055::init(bool force )
{
    //Return if it is already initliazed
    if(is_init && !force)
        return true;
    //If force was passed we want to reset the is_init state
    is_init = false;
    bool success = true;

    //Dummy write
    success &= write_byte(BNO055_PAGE_ID_ADDR, 0,false);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //Set in config mode
    if(!setMode(OPERATION_MODE_CONFIG,false))
    {
        if(debug)
            std::cout << "Could not set into config mode" << std::endl;
        success &= false;
    }
    //Make sure we are on the config page
    success &= write_byte(BNO055_PAGE_ID_ADDR, 0);
    //Read chip id
    uint8_t id= read_byte(BNO055_CHIP_ID_ADDR);

    if(debug)
        std::cout << "Chip id is: 0x" <<std::hex<< (uint16_t)id << std::dec<<std::endl;

    if(id != BNO055_ID)
    {
        std::cout << "You connected the wrong chip - I'm not going to init this one" << std::endl;
        is_init = false;
        return false;
    }
    //Trigger reset
    success &= write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20,false);

    //Wait until reset has been performced
    std::this_thread::sleep_for(std::chrono::milliseconds(650));
    //Set to normal power
    success &= write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    //Use internal oscillator
    success &= write_byte(BNO055_SYS_TRIGGER_ADDR, 0x0);
    //Don't change units
    //write_byte(UNIT_SELECT_REGISTER_ADDRESS, 0);
    //Select ndof mode
    if(!setMode(operation_mode))
    {
        if(debug)
            std::cout << "Selection of NDOF not successful" << std::endl;
        success &= false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    //In case everything went good we set the is_init state = true
    is_init = success;
    return success;
}

BNO055CalibrationStatus BNO055::readCalibrationstatus()
{
    BNO055CalibrationStatus stat;
    uint8_t cal_status = read_byte(BNO055_CALIB_STAT_ADDR);
    stat.system = (cal_status >> 6) & 0x03;
    stat.gyro = (cal_status >> 4 ) & 0x03;
    stat.accel = (cal_status >> 2) & 0x03;
    stat.mag = cal_status & 0x03;
    return stat;
}
BNO055SelftestResult BNO055::runSelftest()
{
    if(debug)
        std::cout << "Doing BNO055 self test" << std::endl;
    setMode(OPERATION_MODE_CONFIG, false);

    const uint8_t sys_trigger = read_byte(BNO055_SYS_TRIGGER_ADDR);
    write_byte(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x01);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const uint8_t self_test = read_byte(BNO055_SELFTEST_RESULT_ADDR);

    setMode(operation_mode);

    const uint8_t status = read_byte(BNO055_SYS_STAT_ADDR);
    const uint8_t error = read_byte(BNO055_SYS_ERR_ADDR);

    if(debug)
        std::cout << "Finished BNO055 self test" << std::endl;

    BNO055SelftestResult result;
    result.selftest = self_test;
    result.error = error;
    result.status = status;

    return  result;
}

BNO055Revision BNO055::readRevision()
{
    BNO055Revision rev;
    rev.accelerometer = read_byte(BNO055_ACCEL_REV_ID_ADDR);
    rev.magnetometer = read_byte(BNO055_MAG_REV_ID_ADDR);
    rev.bootloader = read_byte(BNO055_BL_REV_ID_ADDR);
    const uint8_t sw_lsb = read_byte(BNO055_SW_REV_ID_LSB_ADDR);
    const uint8_t sw_msg = read_byte(BNO055_SW_REV_ID_MSB_ADDR);
    uint16_t sw = (((uint16_t)sw_msg << 8) | (uint16_t)sw_lsb);
    rev.software = sw;

    return  rev;
}

uint8_t BNO055::readError()
{
    return read_byte(BNO055_SYS_ERR_ADDR);
}

uint8_t BNO055::readStatus()
{
    return read_byte(BNO055_SYS_STAT_ADDR);
}

bool BNO055::setMode(const OperationMode &mode)
{
    //Call internal not typed method
    return setMode(static_cast<uint8_t>(mode));
}

std::optional<BNO055_Euler> BNO055::readEuler()
{
    const auto res = read_bytes<6>(EULER_REGISTER_START_ADDRESS);
    if(!res)
        return std::nullopt;
    const std::array<uint8_t,6> response = res.value();

    std::array<int16_t, 3> result = {0};

    for(std::size_t i = 0; i < result.size();i++)
    {
        result[i] = ((response[i*2+1] << 8) | response[i*2]) & 0xFFFF;
    }


    const BNO055_Euler euler_angles = {.heading =(double)result[0] / 16.0,
                                       .roll = (double)result[1] / 16.0,
                                       .pitch = (double)result[2] / 16.0};

    return euler_angles;
}

std::optional<BNO055_Gyro> BNO055::readGyro()
{
    const auto res = read_bytes<6>(BNO055_GYRO_DATA_Z_MSB);
    if(!res)
        return std::nullopt;
    const std::array<uint8_t,6> response = res.value();


    std::array<int16_t, 3> result = {0};

    for(std::size_t i = 0; i < result.size();i++)
    {
        result[i] = ((response[i*2+1] << 8) | response[i*2]) & 0xFFFF;
    }

    const BNO055_Gyro gyro = {.x = (double)result[2] / 16.0,
                              .y = (double)result[1] / 16.0,
                              .z = (double)result[0] / 16.0,
                             };

    return gyro;

}

std::optional<BNO055_Quaternion> BNO055::readQuaternion()
{
    const auto res = read_bytes<8>(BNO055_QUATERNION_DATA_W_LSB_ADDR);
    if(!res)
        return std::nullopt;

    const std::array<uint8_t,8> response = res.value();

    std::array<int16_t, 4> result = {0};

    for(std::size_t i = 0; i < result.size();i++)
    {
        result[i] = ((response[i*2+1] << 8) | response[i*2]) & 0xFFFF;
    }


    const double scale = (1.0 / (1<<14));
    const BNO055_Quaternion quat = {.w = (double)result[0]*scale,
                                    .x =  (double)result[1]*scale,
                                    .y = (double)result[2]*scale,
                                    .z = (double)result[3]*scale};

    return quat;
}

double BNO055::readTemperature()
{
    return read_byte(TEMPERATURE_REGISTER);
}

std::optional<BNO055_LinearAcceleration> BNO055::readLinearAcceleration()
{
    const auto res = read_bytes<6>(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);
    if(!res)
        return std::nullopt;

    const std::array<uint8_t,6> response = res.value();

    std::array<int16_t,3> result = {0};

    for(std::size_t i = 0; i < result.size();i++)
    {
        result[i] = ((response[i*2+1] << 8) | response[i*2]) & 0xFFFF;
    }


    const  BNO055_LinearAcceleration acceleration ={.x = (double)result[0] / 100.0,
                                                    .y = (double)result[1] /100.0,
                                                    .z = (double)result[2] / 100.0
                                                   };

    return acceleration;
}



uint8_t BNO055::read_byte(uint8_t address)
{
    //Workaround needed at sensor init
    const auto res = read_bytes<1>(address);
    if(res)
        return res.value()[0];
    else
        return 0;
}
bool BNO055::write_byte(uint8_t address, uint8_t data, bool ack)
{
    return write_bytes<1>(address, {data},ack);
}

bool BNO055::setMode(uint8_t mode, bool store)
{
    if(store)
        operation_mode = mode;
    bool res = write_byte(BNO055_OPR_MODE_ADDR, mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return res;
}



