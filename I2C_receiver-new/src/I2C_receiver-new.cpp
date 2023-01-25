// I2C Test Harness by Chris Claxton
// Based on the SPI master-slave and i2c examples from the Pico examples repo
// I2C_Receiver

#include <Arduino.h>
#include <Wire.h>

// Debug Signal outputs
#ifdef ARDUINO_ARCH_STM32
    #define DEBUG_PIN2 (D2)
    #define DEBUG_PIN3 (D3)
    #define DEBUG_PIN4 (D4)
    #define DEBUG_PIN5 (D5)
#elseif ARDUINO_ARCH_ESP8266
    #define DEBUG_PIN2 (D5)
    #define DEBUG_PIN3 (D6)
    #define DEBUG_PIN4 (D7)
    #define DEBUG_PIN5 (D8)
#elseif ARDUINO_ARCH_ESP32
    #define DEBUG_PIN2 (2u)
    #define DEBUG_PIN3 (4u)
    #define DEBUG_PIN4 (12u)
    #define DEBUG_PIN5 (13u)
#else
    #define DEBUG_PIN2 (6u)
    #define DEBUG_PIN3 (7u)
    #define DEBUG_PIN4 (8u)
    #define DEBUG_PIN5 (9u)
#endif
#define DEBUG_PIN_INITIAL_STATE (HIGH)

// Serial data output and debugging options:
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages
// Error control settings:
#define DEBUG_SERIAL_DURING_I2C_RECEIVE (true) // Set to false to prevent USB Serial debug output during I2C data reception, using USB serial during I2C reception causes data errors.
#define DEBUG_SERIAL_DURING_I2C_RESPONSE (true) // Set to false to prevent USB Serial debug output during I2C response (sending data back to the master), using USB serial while responding causes data errors on the master.

// I2C Settings:
#define I2C_INSTANCE i2c0 // i2c0 is Wire in Arduino-Pico, valid pins below must be used for each i2c instance
#ifdef ARDUINO_ARCH_ESP32
    #define I2C_SLAVE_SDA_PIN (21u)
    #define I2C_SLAVE_SCL_PIN (22u)
#else
    #define I2C_SLAVE_SDA_PIN (4u)
    #define I2C_SLAVE_SCL_PIN (5u)
#endif
#define I2C_SLAVE_ADDRESS (0x30)
//#define I2C_BAUDRATE      (400000u)  // 400 kHz
#define I2C_BAUDRATE      (1000000u) // 1 MHz

#define USE_SECOND_CORE
#ifdef USE_SECOND_CORE
    #define USE_MUTEX // comment this out to disable mutex protections when using second core.
    auto_init_mutex(gpioMutex);
    auto_init_mutex(sharedVarsMutex);
#endif



#define BUF_LEN         0xFF // 255 byte buffer
uint8_t bufferLength = BUF_LEN;
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

bool ledState = false;
unsigned long startMillis = 0, currentMillis = 0;
unsigned int seconds = 0, lastSeconds = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, receiveErrorCount = 0, sendCounter = 0, sendErrorCount = 0;
unsigned int receivedBytesErrorCount = 0, verifyErrorCount = 0;
bool i2cDataReadFromWireBuffer = false;
unsigned int lastBytesAvailable = 0, lastBytesExpected = 0;
bool dataReady = false;

// underscore vars only to be accessed and modified inside the ISR's
volatile uint8_t _byte = 0, _prevByte = 0;;
volatile unsigned int _byteIndex = 0, _expectedByteCount = 0, _bytesReceived = 0;
volatile unsigned int _byteRequestedIndex = 0;
volatile bool _i2cDataReceiveInProgress = false, _i2cDataRequestInProgress = false;

// Vars accessed accessed and modified both inside and outside ISR's
volatile bool i2cDataReady = false, i2cDataRequested = false, i2cDataRequestCompleted = false;
volatile unsigned int  bytesAvailable = 0, bytesExpectedOrRequested = 0, bytesSent = 0;

void gpioWrite(pin_size_t pinNumber, PinStatus status, bool disableInterrupts = true) {
    #ifdef USE_MUTEX
    if (disableInterrupts) noInterrupts(); // disable interrupts
    mutex_enter_blocking(&gpioMutex);
    #endif
    digitalWrite(pinNumber, status);
    #ifdef USE_MUTEX
    mutex_exit(&gpioMutex);
    if (disableInterrupts) interrupts(); // enable interrupts
    #endif
}

unsigned int readUint(volatile unsigned int *var) {
    unsigned int value = 0;
    #ifdef USE_MUTEX
    noInterrupts(); // disable interrupts
    mutex_enter_blocking(&sharedVarsMutex);
    #endif
    value = *var;
    #ifdef USE_MUTEX
    mutex_exit(&sharedVarsMutex);
    interrupts(); // enable interrupts
    #endif
    return value;
}

void writeUint(volatile unsigned int *var, unsigned int value) {
    #ifdef USE_MUTEX
    noInterrupts(); // disable interrupts
    mutex_enter_blocking(&sharedVarsMutex);
    #endif
    *var = value;
    #ifdef USE_MUTEX
    mutex_exit(&sharedVarsMutex);
    interrupts(); // enable interrupts
    #endif
}

bool readBool(volatile bool *var) {
    unsigned int value = 0;
    #ifdef USE_MUTEX
    noInterrupts(); // disable interrupts
    mutex_enter_blocking(&sharedVarsMutex);
    #endif
    value = *var;
    #ifdef USE_MUTEX
    mutex_exit(&sharedVarsMutex);
    interrupts(); // enable interrupts
    #endif
    return value;
}

void writeBool(volatile bool *var, bool value) {
    #ifdef USE_MUTEX
    noInterrupts(); // disable interrupts
    mutex_enter_blocking(&sharedVarsMutex);
    #endif
    *var = value;
    #ifdef USE_MUTEX
    mutex_exit(&sharedVarsMutex);
    interrupts(); // enable interrupts
    #endif
}


void printBuffer(uint8_t buf[], size_t len) {
    unsigned int i;
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

unsigned int verifyInBuffer(unsigned int page, bool printOnlyFirstError) {
    int errorCount = 0;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        if (in_buf[i] != i + 1) {
            if (errorCount == 0 && printOnlyFirstError) {
                Serial.printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            } else if (!printOnlyFirstError) {
                Serial.printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            }
            errorCount++;
        }
    }
    return errorCount;
}

void clearBuffer(uint8_t buf[], size_t len) {
    for (unsigned int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}

// onReceive interrupt Handler
// Called when the I2C slave has received all of the data from the transmission
void i2c_slave_recv(int byteCount) {
    gpioWrite(DEBUG_PIN2, LOW, false); // signal the start of the interrupt
    // A single byte received is either a command from the master to request data, for example reading a data register from the slave device
    // or for the purposes this test project, a prefix indicating the number of bytes that the sender (I2C master) is going to be send next
    if (byteCount == 1) {
        #ifdef USE_MUTEX
        mutex_enter_blocking(&sharedVarsMutex);
        #endif
        if (Wire.available()) {
            bytesExpectedOrRequested = Wire.read();
        } else {
            bytesExpectedOrRequested = 0;
        }
        // reset the request flags
        bytesSent = 0;
        #ifdef USE_MUTEX
        mutex_exit(&sharedVarsMutex);
        #endif
        _i2cDataRequestInProgress = false;
        _byteRequestedIndex = 0;
        gpioWrite(DEBUG_PIN2, HIGH); // signal the end of the interrupt
        return;
    } else if (byteCount > 1) {
        // The data must be read from the receive buffer inside this handler, this is imposed by the way the Wire API is implemented in Arduino-Pico
        // The Wire receive buffer is cleared inside onIRQ in Wire.cpp after this handler has completed, so is not available outside of this handler.
        int data = 0;
        while (Wire.available()) {
            data = Wire.read(); // returns -1 if no data is available, Wire.available() should prevent this though
            if (_byteIndex < BUF_LEN) {
                // confident that in_buf will not be read in core0 while being written here
                in_buf[_byteIndex] = data; // read into data1 buffer
                _byteIndex++;
            }
        }
        #ifdef USE_MUTEX
        mutex_enter_blocking(&sharedVarsMutex);
        #endif
        bytesAvailable = _byteIndex;
        i2cDataReady = true;
        #ifdef USE_MUTEX
        mutex_exit(&sharedVarsMutex);
        #endif
        _byteIndex = 0;
    }
    gpioWrite(DEBUG_PIN2, HIGH, false); // signal the end of the interrupt
}

// onRequest interrupt Handler
// The handler is called from the I2C ISR, so it must complete quickly. Blocking calls or
// printing to Serial / stdio may interfere with interrupt handling.
void i2c_slave_req() {
    gpioWrite(DEBUG_PIN4, LOW, false); // signal the start of the interrupt
    Wire.write(out_buf[_byteRequestedIndex]); // send the requested data to the sender (master), one byte per interrupt
    _i2cDataRequestInProgress = true;
    #ifdef USE_MUTEX
    mutex_enter_blocking(&sharedVarsMutex);
    #endif
    if (_byteRequestedIndex == 0) i2cDataRequested = true; // only set this once at the start
    _byteRequestedIndex++;

    // check if we have sent all the data that was requested
    if (_byteRequestedIndex >= bytesExpectedOrRequested) {
        bytesSent += _byteRequestedIndex;
        _byteRequestedIndex = 0;
        _i2cDataRequestInProgress = false;
        i2cDataRequestCompleted = true;
    } else if (_byteRequestedIndex >= BUF_LEN) { // fail safe condition to prevent buffer overflow on next interrupt if more data was requested than the size of the buffer
        bytesSent += _byteRequestedIndex; // just roll-over the buffer
        _byteRequestedIndex = 0;
    }
    #ifdef USE_MUTEX
    mutex_exit(&sharedVarsMutex);
    #endif
    gpioWrite(DEBUG_PIN4, HIGH, false); // signal the end of the interrupt
}

// Uses Wire for I2C0 instance
static void setupSlave() {
    // Be sure to use pins labeled I2C0 for Wire and I2C1 for Wire1 on the pinout diagram for your board, otherwise it won’t work.
    // For Wire (I2C0 on the Pico) default pins: PIN_WIRE0_SDA (4u) PIN_WIRE0_SCL  (5u)
    // For Wire1 (I2C1 on the Pico) default pins: PIN_WIRE1_SDA (26u), PIN_WIRE1_SCL (27u)
    // Change these pins before calling Wire.begin() or Wire.begin()
    #ifdef ARDUINO_ARCH_RP2040
        Wire.setSDA(I2C_SLAVE_SDA_PIN);
        Wire.setSCL(I2C_SLAVE_SCL_PIN);
    #elif ARDUINO_ARCH_MBED
        // Arduino MBED core does not support setting the i2c pins like this
    #else
        // Use default pins
    #endif

    // For STM32, the interrupt handlers must be specified before calling Wire.begin(slave_address)
    // For RP2040, ESP8266 and ESP32 this does not matter
    Wire.onReceive(i2c_slave_recv); // i2c receive interrupt
    Wire.onRequest(i2c_slave_req);  // i2c request (send) interrupt
    #ifdef ARDUINO_ARCH_ESP32
        // In the framework-arduinoespressif32/libraries/Wire/src/Wire.cpp there are 2 potentially ambiguous begin functions due to default parameters
        // The 2 begin definitions are: bool begin(int sda=-1, int scl=-1, uint32_t frequency=0); and bool begin(uint8_t slaveAddr, int sda=-1, int scl=-1, uint32_t frequency=0);
        // If we only supply the I2C_SLAVE_ADDRESS, the wrong function definition is chosen (the address is treated as sda pin) so we must specify all parameters
        Wire.begin(I2C_SLAVE_ADDRESS, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN, I2C_BAUDRATE);
    #else
        // Default clock is 100 KHz (with 1k pullup resistors, actually runs at 95.238 KHz)
        Wire.setClock(I2C_BAUDRATE); // Set Clock to 400KHz (Fast Mode) (with 1k pullup resistors, actually runs at 365 KHz)
        // Wire.setClock(1000000); // Set Clock to 1Mhz (Fast Mode Plus) (with 1k pullup resistors, actually runs at 868 KHz)
        Wire.begin(I2C_SLAVE_ADDRESS);  // Configure the i2c bus as slave
    #endif

}

void setup() {
    // Setup Serial Comms
    #ifdef ARDUINO_ARCH_STM32
        Serial.begin(230400); // Max working speed via ST-Link
    #elif ARDUINO_ARCH_ESP8266
        Serial.begin(230400); // Max working speed
    #elif ARDUINO_ESP32_DEV
        Serial.begin(115200); // Max working speed
    #else
        Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    #endif
    int startupDelay = 9;
    for (int i = 1; i <= startupDelay; ++i) {
        Serial.printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        delay(1000);
    }
    Serial.printf("\e[2J\e[H"); // clear screen and go to home position

    Serial.printf("I2C receiver Arduino-Pico example using i2c baud rate: %d \r\n", I2C_BAUDRATE);
    #ifdef ARDUINO_ARCH_RP2040
        Serial.printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
        Serial.printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
        Serial.printf("get_core_num: %u \r\n", get_core_num());
    #endif
    Serial.printf("DEBUG_SERIAL_DURING_I2C_RECEIVE: %s \r\n", DEBUG_SERIAL_DURING_I2C_RECEIVE ? "true" : "false");
    Serial.printf("DEBUG_SERIAL_DURING_I2C_RESPONSE: %s \r\n\r\n", DEBUG_SERIAL_DURING_I2C_RESPONSE ? "true" : "false");

    // Init the onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    // Setup Debug output pins
    pinMode(DEBUG_PIN2, OUTPUT);
    gpioWrite(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN3, OUTPUT);
    gpioWrite(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN4, OUTPUT);
    gpioWrite(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN5, OUTPUT);
    gpioWrite(DEBUG_PIN5, DEBUG_PIN_INITIAL_STATE);

    #ifdef ARDUINO_RASPBERRY_PI_PICO
        //pinMode(23, OUTPUT);
        //gpioWrite(23, HIGH); // Set the SMPS Power Save pin (PS) high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #elif ARDUINO_RASPBERRY_PI_PICO_W
        //pinMode(33, OUTPUT); // On PicoW, the PS pin is connected to the wireless module GPIO2 which is mapped to pin 33 in Arduino-Pico
        //gpioWrite(33, HIGH); // Set the SMPS Power Save pin (PS) high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #endif


    // Setup the I2C hardware
    #ifndef USE_SECOND_CORE
    delayMicroseconds(10); // delay so we can easily see the debug pulse
    gpioWrite(DEBUG_PIN3, LOW); // signal the start of I2C config
    delayMicroseconds(10); // delay so we can easily see the debug pulse
    setupSlave();
    gpioWrite(DEBUG_PIN3, HIGH);  // signal the end of I2C config
    #endif


    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }
    clearBuffer(in_buf, BUF_LEN);

    Serial.printf("I2C Receiver says: After reading I2C data from RX, the value: 0x%02X (%u) (buffer size) and then the following buffer will be written to the sender:\r\n", BUF_LEN, BUF_LEN);
    printBuffer(out_buf, BUF_LEN);
    Serial.printf("\r\n");

    startMillis = millis();
}

#ifdef USE_SECOND_CORE
void setup1() {
    setupSlave();
}
#endif

void loop() {
    // Check if we have received data
    if (readBool(&i2cDataReady)) {
        // gpioWrite(DEBUG_PIN3, LOW);

        // ================================================================
        // IT IS NOT POSSIBLE TO READ FROM THE WIRE RECEIVE BUFFER OUTSIDE OF THE onReceive ISR!
        // The receive buffer indexes used in the Wire.cpp implementation are reset
        // inside the onIRQ function in Wire.cpp after it calls the onReceive callback.
        // ================================================================
        // int data = 0, byteIndex = 0;
        // while (Wire.available()) {
        //     data = Wire.read(); // returns -1 if no data is available, Wire.available() should prevent this though
        //     if (byteIndex < BUF_LEN) {
        //         in_buf[byteIndex] = data; // read into data1 buffer
        //         byteIndex++;
        //     }
        // }
        // bytesAvailable = byteIndex;

        i2cDataReadFromWireBuffer = true; // The data has already been read inside the onReceive ISR, not ideal but the only way.
        // Reset the flag
        writeBool(&i2cDataReady, false);
        //delayMicroseconds(10); // delay so we can easily see the debug pulse
        //gpioWrite(DEBUG_PIN3, HIGH);
    }
    if (i2cDataReadFromWireBuffer && (readBool(&i2cDataRequestCompleted) || DEBUG_SERIAL_DURING_I2C_RESPONSE)) {
        unsigned int bytesAvailableCopy = readUint(&bytesAvailable);
        unsigned int bytesExpectedOrRequestedCopy = readUint(&bytesExpectedOrRequested);
        gpioWrite(LED_BUILTIN, HIGH); // turn on the LED
        gpioWrite(DEBUG_PIN3, LOW);
        if (bytesAvailableCopy == 0) {
            Serial.printf("ERROR!!! Received empty transmission from the Sender (master) page: %u bytesAvailable: %03u                              \r\n", receiveCounter, bytesAvailableCopy);
        } else {
            writeUint(&bytesAvailable, 0);
            lastBytesExpected = bytesExpectedOrRequestedCopy;
            receiveCounter++;
            // Keep track of seconds since start
            currentMillis = millis();
            seconds = (currentMillis - startMillis) / 1000;
            if (seconds - lastSeconds > 0) {
                lastSeconds = seconds;
                //calculate the receive rate, per second
                receiveRate = receiveCounter - lastReceiveCount;
                lastReceiveCount = receiveCounter;
            }

            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                // Reset the previous terminal position if we are not scrolling the output
                if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                    Serial.printf("\e[H"); // move to the home position, at the upper left of the screen
                    Serial.printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                }
                // Print the header info
                Serial.printf("\r\nSeconds: %07u.%03lu       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
                Serial.printf("receiveCounter: %07u         \r\n", receiveCounter);
                Serial.printf("receiveRate: %07u            \r\n", receiveRate);
                Serial.printf("Receive errorCount: %03u         \r\n", receiveErrorCount);
                //Serial.printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * receiveErrorCount / (receiveCounter > 0 ? receiveCounter : 1));
                Serial.printf("receivedBytesErrorCount: %03u         \r\n", receivedBytesErrorCount);
                Serial.printf("sendCounter: %07u             \r\n", sendCounter);
                Serial.printf("Send errorCount: %03u             \r\n", sendErrorCount);
                //Serial.printf("Send FailureRate: %11.7f percent  \r\n", 100.0f * sendErrorCount / (sendCounter > 0 ? sendCounter : 1));
                Serial.printf("Data Received...                                                                \r\n");

                // print data to the serial port
                Serial.printf("I2C Receiver says: read page %u from the sender, received page size: %03u expected: %03u lastExpected: %03u \r\n", receiveCounter, bytesAvailableCopy, bytesExpectedOrRequestedCopy, lastBytesExpected);
                // Write the input buffer out to the USB serial port
                printBuffer(in_buf, BUF_LEN);

                Serial.printf("I2C Receiver says: Verifying received data...                           \r\n");
                if (bytesExpectedOrRequestedCopy != BUF_LEN) {
                    receiveErrorCount++;
                    Serial.printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u                               \r\n", receiveCounter, bytesExpectedOrRequestedCopy, BUF_LEN);
                }
                verifyErrorCount = verifyInBuffer(receiveCounter, false);
                receivedBytesErrorCount += verifyErrorCount;
                // Check that we only record the error once for each receive cycle
                if (bytesExpectedOrRequestedCopy == BUF_LEN && verifyErrorCount > 0) receiveErrorCount++;
            }
            clearBuffer(in_buf, BUF_LEN);
        }
        // Reset the flag
        i2cDataReadFromWireBuffer = false;
        gpioWrite(LED_BUILTIN, LOW); // turn off the LED
        gpioWrite(DEBUG_PIN3, HIGH);
    }
    if (!i2cDataReadFromWireBuffer && readBool(&i2cDataRequested) && (readBool(&i2cDataRequestCompleted) || DEBUG_SERIAL_DURING_I2C_RESPONSE)) {
        unsigned int bytesExpectedOrRequestedCopy = readUint(&bytesExpectedOrRequested);
        delayMicroseconds(10); // delay so we can easily see the debug pulse
        gpioWrite(DEBUG_PIN5, LOW);

        // Leave room for up to 4 previous error report lines not to be overwritten
        for (unsigned int v = 1; v <= 4; ++v) {
            if (v > verifyErrorCount) Serial.printf("\n");
        }
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Receiver says: Responding to Request from the Sender (master) for the Output buffer... (page %u, bytes requested: %03u) \r\n", receiveCounter, bytesExpectedOrRequestedCopy);
        }
        if (bytesExpectedOrRequestedCopy == BUF_LEN) {
            Serial.printf("\r\n");
        } else {
            sendErrorCount++;
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                Serial.printf("ERROR!!! Unexpected number of bytes requested by the Sender (master) for the Output buffer...  (page %u, bytes requested: %03u) \r\n", receiveCounter, bytesExpectedOrRequestedCopy);
            }
        }
        // Reset the flag
        writeBool(&i2cDataRequested,false);
    }
    if (!i2cDataReadFromWireBuffer && !readBool(&i2cDataRequested) && readBool(&i2cDataRequestCompleted)) {
        unsigned int bytesExpectedOrRequestedCopy = readUint(&bytesExpectedOrRequested);
        unsigned int bytesSentCopy = readUint(&bytesSent);
        sendCounter++;
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("I2C Receiver says: Responded to Request from the Sender (master) for the Output buffer  (page %u, bytesSent: %03u)          \r\n", sendCounter, bytesSentCopy);
        }
        if (bytesSentCopy == bytesExpectedOrRequestedCopy) {
            Serial.printf("\r\n");
        } else {
            sendErrorCount++;
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                Serial.printf("ERROR!!! Responding to Request from the Sender (master) for the Output buffer, buffer not completely sent  (page %u, bytesSent: %03u) \r\n", sendCounter, bytesSentCopy);
            }
        }
        // Reset the flag
        writeBool(&i2cDataRequestCompleted, false);
        //Serial.printf("                                                                                                                              \r\n");
        gpioWrite(DEBUG_PIN5, HIGH);
    }
    if (DEBUG_SERIAL_DURING_I2C_RECEIVE && receiveCounter > 0) {
        // Just keep printing something out of USB Serial every 100 uS, this will overlap with i2c receive.
        // 100 uS is around 10 bytes received from i2c at 1MHz, so while receiving 255 bytes we will have decent overlap for testing
        delayMicroseconds(100);
        Serial.print(".");
    }
}
