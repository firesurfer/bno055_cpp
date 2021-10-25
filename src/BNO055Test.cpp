
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

/*
 * Simple test program that initialises the sensor and reads some data from the BNO055
 */
#include <serial/serial.h>
#include"BNO055.h"

#include <thread>
#include <iostream>

int main(int argc, char** argv)
{   
    //Default port name
    std::string port_name = "/dev/ttyUSB0";
    if(argc > 1) //In case a different port was passed
        port_name = std::string(argv[1]);

    auto port = serial::Serial(port_name, 115200);

    std::cout << "Creating BNO055 object" << std::endl;
    //Create BNO055 object and bind flush, bytes avaiable and write, read bytes methods of the serial port
    //This allows replacing the serial port implementation
    using namespace std::placeholders;
    BNO055::SharedPtr imu = std::make_shared<BNO055>(
        [&port](){return port.flush();},
        [&port](){return port.available();},
        [&port](uint8_t * data, std::size_t length){return port.write(data,length);},
        [&port](uint8_t* data, std::size_t length){return port.read(data,length);},
    false
    );
    std::cout << "Starting init" << std::endl;
    //Initialize sensor
    if(!imu->init())
    {
        std::cout << "Error at imu init" << std::endl;
        return 0;
    }

    //Run the selftest
    auto selftest = imu->runSelftest();
    std::cout << "Selftest result: " << std::endl;
    std::cout << "  0x0F is normal: " << std::hex << (int)selftest.selftest  << std::dec<< std::endl;
    if(selftest.status == 0x01)
        std::cout << "System error: " << "0x"<<std::hex << (int)selftest.error << std::dec << std::endl;

    //Read some mroe stuff
    
    auto revision = imu->readRevision();
    std::cout << "Revisions:" << std::endl;
    std::cout << " Accel: " << revision.accelerometer << std::endl;
    std::cout << " Mag: " << revision.magnetometer << std::endl;
    std::cout << " Bootloader: " << revision.bootloader << std::endl;
    std::cout << " Software: " << revision.software << std::endl;


    std::cout << "System status: " << "0x" << std::hex << (int)imu->readStatus() << std::dec << std::endl;

    auto calib = imu->readCalibrationstatus();
    std::cout << "Calibration status: " << std::endl;
    std::cout << " System: " << (int)calib.system << std::endl;
    std::cout << " Gyro: " << (int)calib.gyro << std::endl;
    std::cout << " Accel: " << (int)calib.accel << std::endl;
    std::cout << " Magneto: " << (int)calib.mag << std::endl;



    //In the endless loop we are reading and printing the euler angles
    std::chrono::high_resolution_clock::time_point last_update = std::chrono::high_resolution_clock::now();
    double avg_update_rate = 0;
    while(true)
    {
        const auto result = imu->readEuler();
        if(!result) //Something went wrong when reading so we return with -1
            return -1;

        const auto linaccel_result = imu->readLinearAcceleration();
        if(!linaccel_result)
            return -1;


        //Unpack optional
        const auto euler_angles = result.value();
        const auto linear_accel = linaccel_result.value();
        //Optionally print system status (Halfs update rate)
        //std::cout << "System status: " << "0x" << std::hex << (int)imu->readStatus() << std::dec << std::endl;
        std::cout <<"Temperature: " << imu->readTemperature()<< " Heading: " << euler_angles.heading << " Roll: " << euler_angles.roll << " Pitch: " << euler_angles.pitch <<  "Lin accel: " << linear_accel.x << "," << linear_accel.y << "," << linear_accel.z<< std::endl;

        const auto now = std::chrono::high_resolution_clock::now();
        const auto update_time = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update).count();
        last_update = now;

        avg_update_rate = (avg_update_rate * 0.9) + (0.1*  (1000.0 * 1000.0)/(update_time));
        //In my tests the average update rate is ~200Hz which is higher than the update rate of the sensor in NDOF mode (100Hz)
        std::cout << "Avg update rate: " << avg_update_rate << "Hz" << std::endl;

        //With  1 ms sleep one achieves almost perfect 100Hz update rate in average
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
    std::cout << "exit" << std::endl;
    return 0;
}
