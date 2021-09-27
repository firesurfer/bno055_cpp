
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
 * See https://github.com/adafruit/Adafruit_Python_BNO055 for complete list of definitions
 */

#ifndef BNO055COMMANDDEFINITIONS_H
#define BNO055COMMANDDEFINITIONS_H

#define START_BYTE 0xAA
#define WRITE 0x00
#define READ 0x01


#define BNO055_ID   0xA0
#define BNO055_CHIP_ID_ADDR   0x00
#define WRITE_SUCCESS 0x01
#define WRITE_FAIL 0x03
#define REGMAP_INVALID_ADDRESS 0x04
#define REGMAP_WRITE_DISABLED 0x05
#define WRONG_START_BYTE 0x06
#define BUS_OVER_RUN_ERROR 0x07
#define MAX_LENGTH_ERROR 0x08
#define MIN_LENGHT_ERROR 0x09
#define RECEIVE_CHARACTER_TIMEOUT 0x0A

#define EULER_REGISTER_START_ADDRESS 0x1A
#define EULER_REGISTER_END_ADDRESS 0x1F

#define QUATERNION_REGISTER_START_ADDRESS 0x20
#define QUATERNION_REGISTER_END_ADDRESS 0x2D

#define TEMPERATURE_REGISTER 0x34

#define SELECT_UNIT_CELCIUS 0b00000000
#define SELECT_UNIT_FAHRENHEIT 0b0001000

#define SELECT_UNIT_DEGRESS 0b00000000
#define SELECT_UNIT_RADIANS 0b00000100

#define OPERATION_MODE_CONFIG                 0x00
#define OPERATION_MODE_ACCONLY                0x01
#define OPERATION_MODE_MAGONLY                0x02
#define OPERATION_MODE_GYRONLY                0x03
#define OPERATION_MODE_ACCMAG                 0x04
#define OPERATION_MODE_ACCGYRO                0x05
#define OPERATION_MODE_MAGGYRO                0x06
#define OPERATION_MODE_AMG                    0x07
#define OPERATION_MODE_IMUPLUS                0x08
#define OPERATION_MODE_COMPASS                0x09
#define OPERATION_MODE_M4G                    0x0A
#define OPERATION_MODE_NDOF_FMC_OFF           0x0B
#define OPERATION_MODE_NDOF                   0x0C



#define BNO055_CALIB_STAT_ADDR                0x35
#define BNO055_SELFTEST_RESULT_ADDR           0x36
#define BNO055_INTR_STAT_ADDR                 0x37

#define BNO055_SYS_CLK_STAT_ADDR              0x38
#define BNO055_SYS_STAT_ADDR                  0x39
#define BNO055_SYS_ERR_ADDR                   0x3A


#define UNIT_SELECT_REGISTER_ADDRESS          0x3B
#define OPERATION_MODE_REGISTER_ADDRESS       0x3D



//PAGE0 REGISTER DEFINITION START
#define BNO055_CHIP_ID_ADDR                   0x00
#define BNO055_ACCEL_REV_ID_ADDR              0x01
#define BNO055_MAG_REV_ID_ADDR                0x02
#define BNO055_GYRO_REV_ID_ADDR               0x03
#define BNO055_SW_REV_ID_LSB_ADDR             0x04
#define BNO055_SW_REV_ID_MSB_ADDR             0x05
#define BNO055_BL_REV_ID_ADDR                 0X06

#define BNO055_PAGE_ID_ADDR                   0x07
#define BNO055_OPR_MODE_ADDR                  0x3D
#define BNO055_PWR_MODE_ADDR                  0x3E
#define BNO055_SYS_TRIGGER_ADDR               0x3F

// Power modes
#define POWER_MODE_NORMAL                     0x00
#define POWER_MODE_LOWPOWER                   0x01
#define POWER_MODE_SUSPEND                     0x02


//Linear acceleration data registers
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  0x2D

#define BNO055_QUATERNION_DATA_W_LSB_ADDR    0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR    0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR    0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR    0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR    0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR    0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR    0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR    0X27

#define BNO055_GYRO_DATA_Z_MSB               0x18


#endif // BNO055COMMANDDEFINITIONS_H
