# bn055_cpp

This is a small project that implements a driver for a Bosch BNO055 9-Axis IMU. This implementation uses the serial interface of the sensor.

# Setup

In order to use the example code you need to clone the used [serial port library](https://github.com/wjwwood/serial). 

In the main directory:

```
mkdir -p ext
cd ext
git clone https://github.com/wjwwood/serial
```

Then you can proceed to build the code. 
Again from the main directory:

```
mkdir -p build
cd build
cmake ..
make
```

# Run the example

Your user has to be added to the group `dialout` (`sudo adduser <username> dialout` then logout and log back in)

```
./bno_test <serial port name>
```

Make sure to have wired your BNO055 to use the serial configuration
