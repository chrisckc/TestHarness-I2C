# Test Harness to debug i2c communication issues

### This was created to debug i2c communication issues that i encountered between 2 Raspberry Pi Pico's (RP2040) connected together using i2c

#### This is a PlatformIO workspace, the projects are configured to use the Earle Philhower core for the RP2040 (Arduino-Pico) rather than the official Arduino core which uses the Arm MBED RTOS. An RTOS is unsuitable for the use case i was working on when i encountered this issue.

Intermittent missing data was observed on the receiving Pico during the i2c communications, the error rate seems to depend on the i2c clock speed and other factors such as how much serial communications is taking place and when these communications take place relative to the i2c communications.

When the errors are observed, all of the the data has been confirmed to be correctly sent on the wire by the transmitting Pico, this has been confirmed via an oscilloscope and by the use of it's protocol decoder function.

A lower clock speed reduces the error rate.

pull-up resistors have been changed from 4.7k to 1k, this made no difference.

Serial communications occurring during reception of i2c data increased the error rate.

Serial communications just before to the reception of i2c data also increased the error rate, but less so.

The I2C1 interface is being used, non-default pins for the I2C1 interface are being used, but these are still valid pins for I2C1 according to the datasheet.


