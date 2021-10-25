
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
#include <thread>
#include <iostream>

class BNO055
{
public:
    //Function pointers for flushing, obtaining avaible types, writing and reading bytes
    typedef std::function<void(void)> fn_flush_t;
    typedef std::function<std::size_t(void)> fn_bytes_available_t;
    typedef std::function<void(uint8_t*, std::size_t)> fn_write_bytes_t;
    typedef std::function<void(uint8_t*, std::size_t)> fn_read_bytes_t;
    //Typef for shared and unique ptrs
    typedef std::shared_ptr<BNO055> SharedPtr;
    typedef std::unique_ptr<BNO055> UniquePtr;

    //Operation mode enum
    enum class OperationMode: uint8_t
    {
        Config = OPERATION_MODE_CONFIG,
        AccOnly = OPERATION_MODE_ACCONLY,
        MagOnly = OPERATION_MODE_MAGONLY,
        GyrOnly = OPERATION_MODE_GYRONLY,
        AccMag = OPERATION_MODE_ACCMAG,
        AccGyro = OPERATION_MODE_ACCGYRO,
        MagGyro = OPERATION_MODE_MAGGYRO,
        Amg = OPERATION_MODE_AMG,
        ImuPlus = OPERATION_MODE_IMUPLUS,
        Compass = OPERATION_MODE_COMPASS,
        M4G = OPERATION_MODE_M4G,
        NdofFMCOff = OPERATION_MODE_NDOF_FMC_OFF,
        Ndof = OPERATION_MODE_NDOF
    };

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


    bool setMode(const OperationMode& mode);

private:


    /**
     * @brief ensure_bytes_read - waits until length bytes could be read
     * @param data
     * @param length
     */
    template<std::size_t length>
    void ensure_bytes_read(std::array<uint8_t, length>& data)
    {
        //Wait until there are enough bytes to read
        while(ser_bytes_available() < length)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        ser_read_bytes(data.data(),length);
    }
    /**
     * @brief read_bytes - read length bytes starting for the given register
     * @param address
     * @param length
     * @return
     */
    template<std::size_t length>
    std::optional<std::array<uint8_t,length>> read_bytes(uint8_t address)
    {
        int attempts = 0;
        while(attempts < 5)
        {
            //Flush what we have in the buffer
            ser_flush();

            uint8_t send_buffer[4];
            send_buffer[0] = START_BYTE;
            send_buffer[1] = READ;
            send_buffer[2] = address & 0xFF;
            send_buffer[3] = length & 0xFF;

            //Send request
            ser_write_bytes(send_buffer,4);
            //Read response
            std::array<uint8_t,2> response;
            ensure_bytes_read<2>(response);

            //Retry in case the first read byte is not 0xBB
            if(response[0] != 0xBB)
            {
                if(debug)
                {
                    std::cout << "Reading register 0x" << std::hex << (uint16_t)address << std::dec << " failed" << std::endl;
                    std::cout<< "Response was: 0x" << std::hex << (int)response[0] << " " << (int)response[1]<< std::dec << std::endl;
                }
                attempts++;
            }
            else
            {
                //Second byte tells us how many more bytes we have to read
                const int read_length = response[1];
                if(read_length != length)
                {
                    return std::nullopt;
                }
                //Read the bytes
                std::array<uint8_t, length> res;
                ensure_bytes_read(res);
                return res;
            }
        }
        return std::nullopt;
    }
    /**
     * @brief read_byte - reads a single byte at the given register of the bno
     * @param address
     * @return
     */
    uint8_t read_byte(uint8_t address);

    /**
     * @brief write_bytes - write length bytes starting from the register given by address
     * @param address
     * @param buf
     * @param length
     * @param ack - read ack
     * @return
     */
    template<std::size_t length>
    bool write_bytes(uint8_t address, std::array<uint8_t, length> buf, bool ack= true)
    {
        int attempts = 0;
        while(attempts < 5)
        {
            //Flush what we have in the buffer so far
            ser_flush();
            //Dummy read on what is in rx buffer. Somehow needed at least with the serial lib I used for testing
            uint8_t dummy[ser_bytes_available()];
            ser_read_bytes(dummy, ser_bytes_available());

            //Build request
            std::array<uint8_t,length+4> send_buffer;
            send_buffer[0] = START_BYTE;
            send_buffer[1] = WRITE;
            send_buffer[2] = address & 0xFF;
            send_buffer[3] = length & 0xFF;
            std::copy(buf.begin(),buf.end(), send_buffer.begin()+4);
            //Write the bytes
            ser_write_bytes(send_buffer.data(),length+4);

            //If we dont want to wait for an ackknowledgment simply return
            if(!ack)
            {
                return true;
            }
            //Otherwise try to read response
            std::array<uint8_t,2> response;
            ensure_bytes_read(response);
            if(ack &&(response[0] != 0xEE && response[1] != 0x07))
            {
                if(debug)
                {
                    std::cout << "Writing to register " << std::hex << (uint16_t)address << std::dec << " failed" << std::endl;
                    std::cout<< "Response was: " << std::hex << (int)response[0] << " " << (int)response[1] << std::dec << std::endl;
                }
                attempts++;
            }
            else {
                return true;
            }

        }
        return false;
    }
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
