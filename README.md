# bn055_cpp

This is a small project that implements a driver for a Bosch BNO055 9-Axis IMU. This implementation uses the serial interface of the sensor. The code should not use heap allocated memory at the moment and is therefore also usable for embedded systems. (Appart from the std::this_thread::sleep methods)
At the moment no thread safety is guaranteed.

Build status:
[![CMake](https://github.com/firesurfer/bno055_cpp/actions/workflows/cmake.yml/badge.svg)](https://github.com/firesurfer/bno055_cpp/actions/workflows/cmake.yml)

# Setup

In order to use the example code you need to clone the used [serial port library](https://github.com/wjwwood/serial). 

In the main directory:

```bash
mkdir -p ext
cd ext
git clone https://github.com/wjwwood/serial
```

If you do not have catkin / ROS installed you can use the patched cmake file for there serial lib in the `patch` folder:

```bash
cp patch/CMakeLists.txt ext/serial
```


Then you can proceed to build the code. 
Again from the main directory:

```bash
mkdir -p build
cd build
cmake ..
make
```

## Using a different serial library

You can also wire in a different serial library rather easily. 
Take a look at `BNO055Test.cpp` file.
There you can find the part where the imu object is created:

```c++
    BNO055::SharedPtr imu = std::make_shared<BNO055>(
        [&port](){return port.flush();},
        [&port](){return port.available();},
        [&port](uint8_t * data, std::size_t length){return port.write(data,length);},
        [&port](uint8_t* data, std::size_t length){return port.read(data,length);},
    false
    );
```

You just need to provide the following parameters to the constructor of the BNO055 object in this order:

1. A method to flush the serial port
2. A method that returns the amount of available bytes
3. A method that allows to write a certain amount of data in bytes
4. A method that allows to read a certain amount of data in bytes 

In the example I used lambda function to redirect the calls to the functions of the serial libary (e.g. `port.flush`).

# Run the example

Your user has to be added to the group `dialout` (`sudo adduser <username> dialout` then logout and log back in)

```bash
./bno_test <serial port name>
```

Make sure to have wired your BNO055 to use the serial configuration



