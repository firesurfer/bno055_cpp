
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


#pragma once

/*
 * Defines and routines partially taken and ported to c++ from:
 * https://github.com/adafruit/Adafruit_Python_BNO055
 */

#include <memory>
#include "BNO055DataDefinition.h"
#include "BNO055CommandDefinitions.h"

#include <functional>
#include <optional>

class BNO055
{
public:
    typedef std::function<void(void)> fn_flush_t;
    typedef std::function<std::size_t(void)> fn_bytes_available_t;
    typedef std::function<void(uint8_t*, std::size_t)> fn_write_bytes_t;
    typedef std::function<void(uint8_t*, std::size_t)> fn_read_bytes_t;

    typedef std::shared_ptr<BNO055> SharedPtr;
    /**
     * @brief BNO055
     * @param port - serial port name
     * @param _reset_pin - raspberry can use a hardware pin for resetting the bno0
     */
    BNO055(
        fn_flush_t _flush,
        fn_bytes_available_t _available,
        fn_write_bytes_t _write,
        fn_read_bytes_t _read,
        bool _debug = false
    );
    /**
     * @brief init - initialize the bno055. Needs to be called first
     */
    bool init(bool force = false);
    /**
     * @brief readCalibrationstatus
     * @return The current state of calibration
     */
    BNO055CalibrationStatus readCalibrationstatus();
    /**
     * @brief readEuler
     * @return struct with euler angles
     */
    std::optional<BNO055_Euler> readEuler();

    std::optional<BNO055_Gyro> readGyro();
    /**
     * @brief readQuaternion
     * @return struct with quaternion - normalized
     */
    std::optional<BNO055_Quaternion> readQuaternion();
    /**
     * @brief readTemperature
     * @return chip temperature
     */
    double readTemperature();

    std::optional<BNO055_LinearAcceleration> readLinearAcceleration();
    /**
     * @brief runSelftest
     * @return 0x0F for selftest variable in struct is normal. 0x01 in status indicates error
     */
    BNO055SelftestResult runSelftest();
    /**
     * @brief readRevision
     * @return revision number of the bno
     */
    BNO055Revision readRevision();
    /**
     * @brief readError
     * @return see datasheet
     */
    uint8_t readError();
    /**
     * @brief readStatus
     * @return see datasheet
     */
    uint8_t readStatus();



private:


    /**
     * @brief ensure_bytes_read - waits until length bytes could be read
     * @param data
     * @param length
     */
    void ensure_bytes_read(std::vector<uint8_t> &data, std::size_t length);
    /**
     * @brief read_byte - reads a single byte at the given register of the bno
     * @param address
     * @return
     */
    uint8_t read_byte(uint8_t address);
    /**
     * @brief read_bytes - read length bytes starting for the given register
     * @param address
     * @param length
     * @return
     */
    std::optional<std::vector<uint8_t>> read_bytes(uint8_t address, uint8_t length);
    /**
     * @brief write_bytes - write length bytes starting from the register given by address
     * @param address
     * @param buf
     * @param length
     * @param ack - read ack
     * @return
     */
    bool write_bytes(uint8_t address, uint8_t* buf, uint8_t length, bool ack= true);
    /**
     * @brief write_byte - write a single byte to the given address
     * @param address
     * @param data
     * @param ack
     * @return
     */
    bool write_byte(uint8_t address, uint8_t data, bool ack = true);

    //Function pointers in order to access the serial port 
    fn_bytes_available_t ser_bytes_available;
    fn_read_bytes_t ser_read_bytes;
    fn_write_bytes_t ser_write_bytes;
    fn_flush_t ser_flush;

    bool is_init = false;
    bool setMode(uint8_t mode, bool store = true);

    uint8_t operation_mode = OPERATION_MODE_NDOF;
    /**
     * @brief debug - enable debug messages
     */
    bool debug = false;
};
