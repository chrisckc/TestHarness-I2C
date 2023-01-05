// I2C Test Harness by Chris Claxton
// Based on the SPI master-slave and i2c examples from the Pico examples repo
// I2C_Sender

#include <Arduino.h>
#include <Wire.h>

// Debug Signal outputs
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)
#define DEBUG_PIN_INITIAL_STATE (HIGH)

// Serial data output and debugging options:
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages
// Error control settings:
#define DEBUG_SERIAL_DURING_I2C_REQUEST (false) // Set to false to prevent USB Serial debug output during I2C data reception, using USB serial during I2C reception causes data errors.
// I2C Settings:
#define I2C_INSTANCE i2c1 // i2c1 is Wire1 in Arduino-Pico, valid pins below must be used for each i2c instance
#define I2C_MASTER_SDA_PIN (6u)
#define I2C_MASTER_SCL_PIN (7u)
#define I2C_SLAVE_ADDRESS (0x30)
//#define I2C_BAUDRATE      (400000u)  // 400 kHz
#define I2C_BAUDRATE      (1000000u) // 1 MHz

#define BUF_LEN         0xFF // 255 byte buffer
uint8_t bufferLength = BUF_LEN;
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

bool ledState = false;
unsigned long startMillis = 0, currentMillis = 0;
unsigned int sendCounter = 0, lastSendCount = 0, sendRate = 0;
unsigned int lastSendMillis = 0, sendInterval = 100; // send every 100 milliseconds

unsigned int seconds = 0, lastSeconds = 0;
unsigned int loopCounter = 0, lastLoopCounter = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, receiveErrorCount = 0, incompleteReceiveCount = 0, sendErrorCount = 0;
unsigned int receivedBytesErrorCount = 0;

volatile bool i2cDataReady = false;
volatile unsigned int  bytesAvailable = 0, bytesRequested = 0;

void printBuffer(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            Serial.printf("%02X \r\n", buf[i]);
        else
            Serial.printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        Serial.printf("   \r\n");
    }
}

bool verifyInBuffer(unsigned int page, bool printOnlyFirstError) {
    bool success = true;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        uint8_t inverted = (uint8_t)~i;
        if (in_buf[i] != inverted) {
            receivedBytesErrorCount++;
            if (success && printOnlyFirstError) {
                Serial.printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, inverted, in_buf[i]);
            } else if (!printOnlyFirstError) {
                Serial.printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, inverted, in_buf[i]);
            }
            success = false;
        }
    }
    return success;
}

void clearBuffer(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}

int sendBufferToSlave(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        Serial.printf("I2C Sender says: Sending Output buffer to Receiver (slave Pico)...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    }
    digitalWrite(DEBUG_PIN3, LOW);
    // First send the data length of the buffer so the i2c slave knows what to expect next
    Wire1.beginTransmission(I2C_SLAVE_ADDRESS);
    // The Write function only works if an ACK was received from the transmission of the address by beginTransmission(address)
    Wire1.write(length);
    // Transmissions are buffered (up to 128 bytes) and only performed on endTransmission, as is standard with modern Arduino Wire implementations.
    uint8_t result1 = Wire1.endTransmission();
    if (result1 > 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Sender says: ERROR!!! Could not send Output buffer page %u, return value: %d Couldn't write to i2c slave, please check your wiring! \r\n", sendCounter, result1);
        }
        digitalWrite(DEBUG_PIN3, HIGH);
        return 0;
    }
    // Now send the buffer
    Wire1.beginTransmission(I2C_SLAVE_ADDRESS);
    // The Write function only works if an ACK was received from the transmission of the address by beginTransmission(address)
    Wire1.write(out_buf, length);
    // Transmissions are buffered (up to 128 bytes) and only performed on endTransmission, as is standard with modern Arduino Wire implementations.
    uint8_t result2 = Wire1.endTransmission();
    if (result2 > 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Sender says: ERROR!!! Could not send Output buffer page %u, return value: %d Couldn't write to i2c slave, please check your wiring! \r\n", sendCounter, result2);
        }
        digitalWrite(DEBUG_PIN3, HIGH);
        return 0;
    }
    digitalWrite(DEBUG_PIN3, HIGH);
    sendCounter++;
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        if (DEBUG_SERIAL_DURING_I2C_REQUEST) {
            // If USB Serial is sent here, it may still be on its way out of a FIFO while the code continues from from here onto the I2C request and read operation (requestBufferFromSlave() )
            Serial.printf("I2C Sender says: Output buffer page %u sent, buffer size: %03u \r\n", sendCounter, length);
        }
    }
    return length;
}

int requestBufferFromSlave(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        Serial.printf("I2C Sender says: Requesting buffer page %u from Receiver (slave Pico) ...  (requesting buffer size: %03u) \r\n", sendCounter, length);
    }
    bytesAvailable = 0;
    digitalWrite(DEBUG_PIN2, LOW);

    // First write the command to the Receiver indicating the size of the buffer we want to read (length), set timeout to 10 ms
    // SDK method:
    //int byteCount = i2c_write_timeout_us(I2C_INSTANCE, I2C_SLAVE_ADDRESS, &length, 1, true, 1000 * 10); // Important that "nostop" is set to true here for a Request operation
    // A negative return value from the above function indicates an error:
    // Return values, from SDK: PICO_OK = 0, PICO_ERROR_NONE = 0, PICO_ERROR_TIMEOUT = -1 , PICO_ERROR_GENERIC = -2 , PICO_ERROR_NO_DATA = -3 , PICO_ERROR_NOT_PERMITTED = -4 , PICO_ERROR_INVALID_ARG = -5 , PICO_ERROR_IO = -6

    Wire1.beginTransmission(I2C_SLAVE_ADDRESS);
    int byteCount = Wire1.write(length);
    uint8_t result1 = Wire1.endTransmission(false); // Important that stopBit is set to false, this is the inverse of "nostop" in the Pico SDK function used underneath
    if (byteCount <= 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Sender says: ERROR!!! Could not write the command to Request buffer page %u, return value: %d (Couldn't read from i2c slave, please check your wiring!) \r\n", sendCounter, byteCount);
        }
        digitalWrite(DEBUG_PIN2, HIGH);
        return 0;
    }

    // Now read the buffer that the Receiver (slave) should be sending back in response to the read Request
    // SDK method:
    //byteCount = i2c_read_timeout_us(I2C_INSTANCE, I2C_SLAVE_ADDRESS, in_buf, length, false, 1000 * 10); // set timeout to 10 ms
    byteCount = Wire1.requestFrom(I2C_SLAVE_ADDRESS, length);
    if (byteCount <= 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Sender says: ERROR!!! Could not read the Request for buffer page %u, return value: %d (Couldn't read from i2c slave, please check your wiring!) \r\n", sendCounter, byteCount);
        }
        digitalWrite(DEBUG_PIN2, HIGH);
        return 0;
    }
    // Retrieve the data from the Wire buffer
    int byteIndex = 0;
    while (Wire1.available()) {
        if (byteIndex < BUF_LEN) {
             in_buf[byteIndex] = Wire1.read();; // read into data1 buffer, returns -1 if no data is available, protected by Wire1.available()
             byteIndex++;
        }
    }
    digitalWrite(DEBUG_PIN2, HIGH);
    bytesAvailable = byteCount;
    i2cDataReady = true;

    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        Serial.printf("I2C Sender says: Buffer page %u read from the Receiver (slave Pico), received buffer size: %03u expected: %03u  \r\n", sendCounter, byteCount, length);
    }
    return byteCount;
}

void resetAllRxVars() {
    i2cDataReady = false;
    bytesAvailable = 0;
}

// Uses Wire1 for I2C1 instance
static void setupMaster() {
    // Be sure to use pins labeled I2C0 for Wire and I2C1 for Wire1 on the pinout diagram for your board, otherwise it won’t work.
    // For Wire (I2C0 on the Pico) default pins: PIN_WIRE0_SDA (4u) PIN_WIRE0_SCL  (5u)
    // For Wire1 (I2C1 on the Pico) default pins: PIN_WIRE1_SDA (26u), PIN_WIRE1_SCL (27u)
    // Change these pins before calling Wire1.begin() or Wire1.begin()
    #ifdef ARDUINO_ARCH_MBED
        // Arduino MBED core does not support setting the i2c pins like this
    #else
        Wire1.setSDA(I2C_MASTER_SDA_PIN);
        Wire1.setSCL(I2C_MASTER_SCL_PIN);
    #endif
    // Default clock is 100 KHz (with 1k pullup resistors, actually runs at 95.238 KHz)
    Wire1.setClock(I2C_BAUDRATE);
    Wire1.begin(); // Configure the i2c bus as master
}

void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART

    int startupDelay = 10;
    for (int i = 1; i <= startupDelay; ++i) {
        Serial.printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    Serial.printf("\e[2J\e[H"); // clear screen and go to home position

    Serial.printf("I2C Sender Arduino-Pico example using i2c baud rate: %d \r\n", I2C_BAUDRATE);
    Serial.printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    Serial.printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    Serial.printf("get_core_num: %u \r\n", get_core_num());
    Serial.printf("DEBUG_SERIAL_DURING_I2C_REQUEST: %s \r\n\r\n", DEBUG_SERIAL_DURING_I2C_REQUEST ? "true" : "false");

    // Init the onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    // Setup Debug output pins
    pinMode(DEBUG_PIN2, OUTPUT);
    digitalWrite(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN3, OUTPUT);
    digitalWrite(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN4, OUTPUT);
    digitalWrite(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN5, OUTPUT);
    digitalWrite(DEBUG_PIN5, DEBUG_PIN_INITIAL_STATE);

    #ifdef ARDUINO_RASPBERRY_PI_PICO
        //pinMode(23, OUTPUT);
        //digitalWrite(23, HIGH); // Set the SMPS Power Save pin (PS) high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #elif ARDUINO_RASPBERRY_PI_PICO_W
        //pinMode(33, OUTPUT); // On PicoW, the PS pin is connected to the wireless module GPIO2 which is mapped to pin 33 in Arduino-Pico
        //digitalWrite(33, HIGH); // Set the SMPS Power Save pin (PS) high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #endif

    sleep_us(10); // delay so we can easily see the debug pulse
    digitalWrite(DEBUG_PIN3, LOW); // signal the start of I2C config

    // Setup the I2C hardware
    setupMaster();
    digitalWrite(DEBUG_PIN3, HIGH);  // signal the end of I2C config

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // The values should be: {0x01, 0x02, 0x03...}
        out_buf[i] = i + 1;
    }
    clearBuffer(in_buf, BUF_LEN);

    Serial.printf("I2C Sender says: The value: 0x%02X (%u) (buffer size) followed immediately by the buffer printed below will be sent to the receiver every: %u ms\r\n", BUF_LEN, BUF_LEN, sendInterval);
    printBuffer(out_buf, BUF_LEN);
    Serial.printf("\r\n");
    Serial.printf("The value 0x%02X (%u) is expected to be returned followed by a reversed version of the above buffer\r\n\r\n", BUF_LEN, BUF_LEN);

    startMillis = to_ms_since_boot(get_absolute_time());
}

void loop() {
    loopCounter++;
    // Keep track of seconds since start
    currentMillis = to_ms_since_boot(get_absolute_time());
    seconds = (currentMillis - startMillis) / 1000;
    if (seconds - lastSeconds > 0) {
        lastSeconds = seconds;
        // calculate the loop rate, per second
        lastLoopCounter = loopCounter;
        loopCounter = 0;
        // calculate the send rate, per second
        sendRate = sendCounter - lastSendCount;
        lastSendCount = sendCounter;
        //calculate the receive rate, per second
        receiveRate = receiveCounter - lastReceiveCount;
        lastReceiveCount = receiveCounter;
    }

    // Send data every sendInterval
    if (currentMillis >= lastSendMillis + sendInterval) {
        lastSendMillis = currentMillis;
        digitalWrite(LED_BUILTIN, HIGH); // turn on the LED
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            if ((receiveCounter + incompleteReceiveCount) < sendCounter) {
                incompleteReceiveCount++;
                Serial.printf("ERROR!!! The page %u response was incomplete!!! bytesRequested: %03u and bytesAvailable: %03u should equal the Buffer Length: %03u\r\n", sendCounter, bytesRequested, bytesAvailable, BUF_LEN);
                printBuffer(in_buf, BUF_LEN);
                Serial.printf("I2C Sender says: ERROR!!! Received incomplete buffer!!! (printed above) \r\n");
                bool verifySuccess = verifyInBuffer(sendCounter, true);
                if (!verifySuccess) receiveErrorCount++;
                clearBuffer(in_buf, BUF_LEN);
                resetAllRxVars(); // recover from this anomaly
            }
            // Reset the previous terminal position if we are not scrolling the output
            if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                Serial.printf("\e[H"); // move to the home position, at the upper left of the screen
                Serial.printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            }
            // Print the header info
            Serial.printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
            Serial.printf("LoopRate: %07u            \r\n", lastLoopCounter); // how many loops per second
            Serial.printf("sendCounter: %07u         \r\n", sendCounter);
            Serial.printf("sendRate: %07u            \r\n", sendRate);
            Serial.printf("Send errorCount: %03u         \r\n", sendErrorCount);
            Serial.printf("Send FailureRate: %11.7f percent  \r\n", 100.0f * sendErrorCount / (sendCounter > 0 ? sendCounter : 1));
            Serial.printf("receiveCounter: %07u         \r\n", receiveCounter);
            Serial.printf("receiveRate: %07u            \r\n", receiveRate);
            Serial.printf("Receive errorCount: %03u             \r\n", receiveErrorCount);
            Serial.printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * receiveErrorCount / (sendCounter > 0 ? sendCounter : 1));
            Serial.printf("Receive incompleteReceiveCount: %03u         \r\n", incompleteReceiveCount);
            Serial.printf("receivedBytesErrorCount: %03u         \r\n", receivedBytesErrorCount);
        }
        // Send the Buffer to the Receiver
        int bytesSent = sendBufferToSlave(BUF_LEN);
        if (bytesSent < BUF_LEN) {
            sendErrorCount++;
        }

        // When using Arduino-Pico, we need a delay here to allow time for the receiver to read the received data buffer from the Wire API
        // for a 255 byte buffer this can take a bit of time
        // If we send a request for data too soon, it messes up the receiver logic as implemented
        // When using the SDK, this is not an issue as the receiver is responding to each byte received due to direct use of the receive ISR, so we already have the data where we want it at the end of the transmission.
        delayMicroseconds(80);

        // Request the buffer from the Receiver
        bytesRequested = BUF_LEN;
        int bytesReceived = requestBufferFromSlave(bytesRequested);
        if (bytesReceived != bytesRequested) {
            //receiveErrorCount++; // do this below instead
        }
        // The results are checked below, although they could be checked here instead.

        digitalWrite(LED_BUILTIN, LOW); // turn off the LED
    }

    // Check if we have all the expected data from the i2c read request
    if (i2cDataReady) {
        receiveCounter++;
        digitalWrite(DEBUG_PIN3, LOW);
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            // Write the input buffer out to the USB serial port
            printBuffer(in_buf, BUF_LEN);
            Serial.printf("I2C Sender says: Verifying received data...                                         \r\n");
            if (bytesAvailable != BUF_LEN) {
                receiveErrorCount++;
                Serial.printf("ERROR!!! page: %u bytesAvailable: %03u should equal the Buffer Length: %03u\r\n", receiveCounter, bytesAvailable, BUF_LEN);
            }
            bool verifySuccess = verifyInBuffer(receiveCounter, false);
            // Check that we only record the error once for each receive cycle
            if (bytesAvailable == BUF_LEN && !verifySuccess) receiveErrorCount++;
        }
        clearBuffer(in_buf, BUF_LEN);
        i2cDataReady = false;
        bytesAvailable = 0;
        digitalWrite(DEBUG_PIN3, HIGH);
    }
}
