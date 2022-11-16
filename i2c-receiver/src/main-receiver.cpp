
#include <Arduino.h>
#include <Wire.h>

// Modifed to also compile for Arduino MBED core, added new entry in PlatformIO.ini
#ifdef ARDUINO_ARCH_MBED
    REDIRECT_STDOUT_TO(Serial); // https://github.com/platformio/platform-raspberrypi/issues/24
#endif
#ifdef ARDUINO_ARCH_MBED
    #define SERIAL_PRINTF(...) printf(__VA_ARGS__)
#else
    #define SERIAL_PRINTF(...) Serial.print(__VA_ARGS__)
#endif

// Debug Signal outputs, to allow us to observe any error conditions on the scope
#define DEBUG_PIN2 (6u)
#define DEBUG_PIN3 (7u)
#define DEBUG_PIN4 (8u)
#define DEBUG_PIN5 (9u)

#define SLAVE_PICO_ADDRESS 0x30
// Modified to also work using the Arduino MBED core which does not support i2c1 Wire1, using i2c0 instead
#define I2C_SDA_PIN (4u) // i2c0 Wire default
#define I2C_SCL_PIN (5u) // i2c0 Wire default
// #define I2C_SDA_PIN (26u) // used for i2c1 Wire1
// #define I2C_SCL_PIN (27u) // used for i2c1 Wire1

#define PRINT_DEBUG_DATA false
#define SERIAL_DURING_I2C true

bool ledState = false;
int debugPinInitialState = HIGH;

volatile unsigned int  dataReceivedCounter = 0, lastDataReceivedBytes = 0, lastByteCounter = 0;
volatile unsigned int dataReceived1Counter = 0, dataReceived1Bytes = 0, dataReceived2Counter = 0, dataReceived2Bytes = 0;
volatile bool dataReceived = false, dataReceived1 = false, dataReceived2 = false, allDataReceived = false;
unsigned int receiveRate = 0, lastDataReceivedCounter = 0;
unsigned int dataReceived1SuccessCount = 0, dataReceived2SuccessCount = 0;
unsigned int dataReceived1FailureCount = 0, dataReceived2FailureCount = 0;
unsigned int dataNotAvailableCount = 0, dataNotAvailable1Count = 0, dataNotAvailable2Count = 0;
bool dataDecoded1 = false, dataDecoded2 = false;

// Code from the transmitter sketch, used for verification
#define TEST_DATA1_SIZE 21  // 21 floats will need 21 * 4 = 84 bytes to be sent
float testData1[TEST_DATA1_SIZE] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
#define TEST_DATA2_SIZE 12  // 12 floats will need 12 * 4 = 48 bytes to be sent
float testData2[TEST_DATA2_SIZE] = {3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2};

// The transmit buffer is 128 bytes, not sure that the receive buffer size is??
#define RECV_BUFF_MAX_SIZE (byte)128
#define DATA1_SIZE (84) // use the same size as originally encountered
#define DATA2_SIZE (48) // use the same size as originally encountered
char receiveBuff1[DATA1_SIZE]; // I2C Receive buffer for first transmission
char receiveBuff2[DATA2_SIZE]; // I2C Receive buffer for second transmission

static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
float decodedFloatValues1[DATA1_SIZE / 4]; // 21 floats
float decodedFloatValues2[DATA2_SIZE / 4]; // 12 floats



#ifdef ARDUINO_ARCH_MBED
// Modifed version of onReceive interrupt Handler for debugging issues with ArduinoCore-MBED
// The transmission rate needs to be dropped to no more than 3 transmission pairs per second as this is all that the MBED RTOS can handle
// The data from both of the i2c transmissions wou;d be expected to be available here due to the large delay between end of transmission and this interrupt firing (264mS)
// This is only being fired once despite there being 2 separate transmissions, with the second being 1mS after the other, this is much less than the interrupt response time.
void recv(int byteCount) {
    digitalWrite(DEBUG_PIN2, LOW); // signal the start of the interrupt
    digitalWrite(LED_BUILTIN, HIGH);
    //if (byteCount > RECV_BUFF_MAX_SIZE) byteCount = RECV_BUFF_MAX_SIZE;
    dataReceived = true;
    dataReceivedCounter++;
    lastDataReceivedBytes = byteCount;
    int i = 0;
    while (Wire.available() && i < DATA1_SIZE + DATA2_SIZE) {
        int data = Wire.read(); // returns -1 if no data is available
        if (data > -1) {
            if (i < DATA1_SIZE) {
                receiveBuff1[i] = data; // read into data1 buffer
            } else {
                receiveBuff2[i - DATA1_SIZE] = data; // read into data2 buffer
            }
        } else {
            dataNotAvailableCount++;
        }
        i++;
    }
    lastByteCounter = i;
    if (i >= DATA1_SIZE) {
        dataReceived1Counter++;
        dataReceived1Bytes = DATA1_SIZE; // can't tell how many of the bytes were from the data1 transmission
        dataReceived1 = true;
        allDataReceived = false; // Data1 arrives first
    }
    if (i >= DATA1_SIZE + DATA2_SIZE) {
        dataReceived2Counter++;
        dataReceived2Bytes = abs(byteCount - DATA1_SIZE); // sometime the bytes are less than DATA1_SIZE
        dataReceived2 = true;
        allDataReceived = true;
    }

    while (Wire.available()) {
        Wire.read(); // clear buffer of any excess data
    }
    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_PIN2, HIGH); // signal the end of the interrupt
}
#else

// onReceive interrupt Handler
// Called when the I2C slave gets written to
// This code takes 36 uS for 84 bytes, 21 uS for 48 bytes
// delay from end of transmission from master to interrupt firing is about 230 uS
// Interval between 2 consecutive sends from the master is 1000uS
void recv(int byteCount) {
    digitalWrite(DEBUG_PIN2, LOW); // signal the start of the interrupt
    digitalWrite(LED_BUILTIN, HIGH);
    //if (byteCount > RECV_BUFF_MAX_SIZE) byteCount = RECV_BUFF_MAX_SIZE;
    dataReceived = true;
    dataReceivedCounter++;
    lastDataReceivedBytes = byteCount;
    int i;
    if (byteCount > DATA2_SIZE) { // New checking method for data1, as data1 is somewhat larger than data2, allows capture if less than expected size
        allDataReceived = false; // Data1 arrives first
        dataReceived1Counter++;
        dataReceived1Bytes = byteCount;
        for (i = 0; i < DATA1_SIZE; i++) { // UPDATED: now reads the expected number of bytes instead of byteCount
            int data = Wire.read(); // returns -1 if no data is available
            if (data > -1) {
                receiveBuff1[i] = data; // read into data1 buffer
            } else {
                dataNotAvailable1Count++;
            }
        }
        dataReceived1 = true;
    } else if (byteCount <= DATA2_SIZE) { // also allows capture if less than expected size
        dataReceived2Counter++;
        dataReceived2Bytes = byteCount;
        for (i = 0; i < DATA2_SIZE; i++) { // UPDATED: now reads the expected number of bytes instead of byteCount
            int data = Wire.read(); // returns -1 if no ddata is available
            if (data > -1) {
               receiveBuff2[i] = data; // read into data2 buffer
            } else {
                dataNotAvailable2Count++;
            }
        }
        dataReceived2 = true;
        allDataReceived = true;
    }
    lastByteCounter = i;
    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_PIN2, HIGH); // signal the end of the interrupt
}
#endif

// Called when the I2C slave is read from
void req() {
}

void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    // Wait for Serial Comms or Timeout after 5 seconds
    while (!Serial && millis() < 5000);

    pinMode(LED_BUILTIN, OUTPUT);

    #ifdef ARDUINO_RASPBERRY_PI_PICO
        pinMode(23, OUTPUT);
        digitalWrite(23, HIGH); // Set the SMPS Power Save pin high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #endif

    // Be sure to use pins labeled I2C0 for Wire and I2C1 for Wire1 on the pinout diagram for your board, otherwise it won’t work.
    // For Wire (I2C0 on the Pico) default pins: PIN_WIRE0_SDA (4u) PIN_WIRE0_SCL  (5u)
    // For Wire1 (I2C1 on the Pico) default pins: PIN_WIRE1_SDA (26u), PIN_WIRE1_SCL (27u)
    // Change these pins before calling Wire1.begin() or Wire1.begin()
    #ifdef ARDUINO_ARCH_MBED
        // Arduino MBED core does not support setting the i2c pins like this
    #else
        Wire.setSDA(I2C_SDA_PIN);
        Wire.setSCL(I2C_SCL_PIN);
    #endif
    // Default clock is 100 KHz (with 1k pullup resistors, actually runs at 95.238 KHz)
    Wire.setClock(400000); // Set Clock to 400KHz (Fast Mode) (with 1k pullup resistors, actually runs at 365 KHz)
    // Wire.setClock(1000000); // Set Clock to 1Mhz (Fast Mode Plus) (with 1k pullup resistors, actually runs at 868 KHz)
    Wire.begin(SLAVE_PICO_ADDRESS); // Join the i2c bus as slave
    Wire.onReceive(recv);           // i2c receive interrupt
    Wire.onRequest(req);            // i2c request (send) interrupt

    // Setup Debug output pins
    pinMode(DEBUG_PIN2, OUTPUT);
    digitalWrite(DEBUG_PIN2, debugPinInitialState);
    pinMode(DEBUG_PIN3, OUTPUT);
    digitalWrite(DEBUG_PIN3, debugPinInitialState);
    pinMode(DEBUG_PIN4, OUTPUT);
    digitalWrite(DEBUG_PIN4, debugPinInitialState);
    pinMode(DEBUG_PIN5, OUTPUT);
    digitalWrite(DEBUG_PIN5, debugPinInitialState);
    delay(5000);
    SERIAL_PRINTF("Pi Pico i2c receiver:\r\n");
    SERIAL_PRINTF("Ready to Receive Data...\r\'n");
    if (dataReceivedCounter > 0) {
        dataReceived = false;
        dataReceivedCounter = 0;
    }
    if (dataReceived1Counter > 0) {
        dataReceived1 = false;
        dataReceived1Counter = 0;
    }
    if (dataReceived2Counter > 0) {
        dataReceived2 = false;
        dataReceived2Counter = 0;
    }
}


void clearReceiveBuffer1() {
  for (unsigned int i = 0; i < DATA1_SIZE; i++) receiveBuff1[i] = 0;
}

void clearReceiveBuffer2() {
  for (unsigned int i = 0; i < DATA2_SIZE; i++) receiveBuff2[i] = 0;
}

void clearFloatArray1() {
  for (unsigned int i = 0; i < DATA1_SIZE / 4; i++) decodedFloatValues1[i] = 0;
}

void clearFloatArray2() {
  for (unsigned int i = 0; i < DATA2_SIZE / 4; i++) decodedFloatValues2[i] = 0;
}

void decodeData(char *byteArrPointer, unsigned int dataReceivedBytes, float *floatArrayPointer) {
    unsigned int i;
    if (PRINT_DEBUG_DATA) SERIAL_PRINTF("Received Bytes: %d\r\n", dataReceivedBytes);
    for (i = 0; i < dataReceivedBytes; i += 4) {
        //unsigned int8_t byte = *(byteArrPointer + i);
        float f;
        memcpy(&f, byteArrPointer + i, 4);
        if (PRINT_DEBUG_DATA) SERIAL_PRINTF("f%d: %+8.3f \r\n", i / 4, f);
        *(floatArrayPointer + (i / 4)) = f;
    }
    if (PRINT_DEBUG_DATA) SERIAL_PRINTF("end: %d \r\n\r\n", i);
}

void verifyData1(int counter1) {
    SERIAL_PRINTF("dr1Count:%d Verifying decodedFloatValues1:                                                             \r\n", counter1);
    for (int i = 0; i < TEST_DATA1_SIZE; i++) {
        if (testData1[i] != decodedFloatValues1[i]) {
            SERIAL_PRINTF("                                                      dr1Count:%d floatValues1 Error! Index: %d expectedFloat: %3.1f receivedFloat: %3.1f \r\n", counter1, i, testData1[i], decodedFloatValues1[i]);
        }
    }
    //SERIAL_PRINTF("                                                                                                         \r\n");
}

void verifyData2(int counter2) {
    SERIAL_PRINTF("dr2Count:%d Verifying decodedFloatValues2:                                                             \r\n", counter2);
    for (int i = 0; i < TEST_DATA2_SIZE; i++) {
        if (testData2[i] != decodedFloatValues2[i]) {
            SERIAL_PRINTF("                                                      dr2Count:%d floatValues2 Error! Index: %d expectedFloat: %3.1f receivedFloat: %3.1f \r\n", counter2, i, testData2[i], decodedFloatValues2[i]);
        }
    }
    //SERIAL_PRINTF("                                                                                                         \r\n");
}

void printData(int counter1, int counter2) {
    SERIAL_PRINTF("dr1Count:%d decodedFloatValues1:                                                        \r\n", counter1);
    for (int i = 0; i < DATA1_SIZE / 4; i++) {
        SERIAL_PRINTF("%3.1f,", decodedFloatValues1[i]);
    }
    SERIAL_PRINTF("\r\ndr2Count:%d decodedFloatValues2:                                                        \r\n", counter2);
    for (int i = 0; i < DATA2_SIZE / 4; i++) {
        SERIAL_PRINTF("%3.1f,", decodedFloatValues2[i]);
    }
    SERIAL_PRINTF("\r\n                                                                            \r\n");
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;
    if (loopCounter > 1) {
        if (dataReceived) {
            #ifdef ARDUINO_ARCH_MBED
                if (SERIAL_DURING_I2C) SERIAL_PRINTF("\r\ndrCount: %07u lastDataReceivedBytes: %u  lastByteCounter: %u          \r\n", dataReceivedCounter, lastDataReceivedBytes, lastByteCounter);
            #endif
            dataReceived = false; // Reset the Data Received flag after it has been collected
        }
        if (dataReceived1 || dataReceived2) {
            digitalWrite(DEBUG_PIN3, LOW); // signal the start of the work for debug purposes
            if (dataReceived1) {
                if (dataReceived1Bytes == DATA1_SIZE) {
                    dataReceived1SuccessCount++;
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("dr1: dr1Count: %07u Success, received: %u bytes                 | ", dataReceived1Counter, dataReceived1Bytes);
                } else {
                    dataReceived1FailureCount++;
                    digitalWrite(DEBUG_PIN4, LOW); // debug purposes
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("dr1: dr1Count: %07u Error!!, received: %u expected: %u bytes! | ", dataReceived1Counter, dataReceived1Bytes, DATA1_SIZE);
                    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
                    digitalWrite(DEBUG_PIN4, HIGH); // debug purposes
                }
                if (dataNotAvailable1Count > 0) {
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("                                                   Wire.read() returned no data! dataNotAvailable1Count: %u  | ", dataNotAvailable1Count);
                }
                decodeData(receiveBuff1, dataReceived1Bytes, decodedFloatValues1);
                dataDecoded1 = true;
                clearReceiveBuffer1();
                dataReceived1 = false; // Reset the Data Received flag after it has been collected
                dataNotAvailable1Count = 0;
            }
            if (dataReceived2 && !dataReceived1) {
                if (dataReceived2Bytes == DATA2_SIZE) {
                    dataReceived2SuccessCount++;
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("dr2: dr2Count: %07u Success, received: %u bytes                 ", dataReceived2Counter, dataReceived2Bytes);
                } else {
                    dataReceived2FailureCount++;
                    digitalWrite(DEBUG_PIN4, LOW); // debug purposes
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("dr2: dr2Count: %07u Error!!, received: %u expected: %u bytes! ", dataReceived2Counter, dataReceived2Bytes, DATA2_SIZE);
                    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
                    digitalWrite(DEBUG_PIN4, HIGH); // debug purposes
                }
                if (dataNotAvailable2Count > 0) {
                    if (SERIAL_DURING_I2C) SERIAL_PRINTF("  Wire.read() returned no data! dataNotAvailable2Count: %u ", dataNotAvailable2Count);
                }
                SERIAL_PRINTF("\r\n");
                decodeData(receiveBuff2, dataReceived2Bytes, decodedFloatValues2);
                dataDecoded2 = true;
                clearReceiveBuffer2();
                dataReceived2 = false; // Reset the Data Received flag after it has been collected
                dataNotAvailable2Count = 0;
            }
            digitalWrite(DEBUG_PIN3, HIGH); // signal the end of the work for debug purposes
        }
        #ifdef ARDUINO_ARCH_MBED
        if (dataDecoded1 || dataDecoded2) {
        #else
        if (allDataReceived && dataDecoded1 && dataDecoded2) {
        #endif
            delay(1);                      // 1mS delay here to make it easier to see DEBUG_PIN on the scope
            digitalWrite(DEBUG_PIN3, LOW); // debug purposes
            if (dataReceived1Bytes == DATA1_SIZE) {
                if (!SERIAL_DURING_I2C) SERIAL_PRINTF("dr1: dr1Count: %07u Success, received: %u bytes                 | ", dataReceived1Counter, dataReceived1Bytes);
            } else {
                if (!SERIAL_DURING_I2C) SERIAL_PRINTF("dr1: dr1Count: %07u Error!!, received: %u expected: %u bytes! | ", dataReceived1Counter, dataReceived1Bytes, DATA1_SIZE);
            }
            if (dataReceived2Bytes == DATA2_SIZE) {
                if (!SERIAL_DURING_I2C) SERIAL_PRINTF("dr2: dr2Count: %07u Success, received: %u bytes                 \r\n", dataReceived2Counter, dataReceived2Bytes);
            } else {
                if (!SERIAL_DURING_I2C) SERIAL_PRINTF("dr2: dr2Count: %07u Error!!, received: %u expected: %u bytes! \r\n", dataReceived2Counter, dataReceived2Bytes, DATA2_SIZE);
            }
            if (dataDecoded1) verifyData1(dataReceived1Counter);
            if (dataDecoded2) verifyData2(dataReceived2Counter);
            if (PRINT_DEBUG_DATA) printData(dataReceived1Counter, dataReceived2Counter);
            //printData(dataReceived1Counter, dataReceived2Counter);
            SERIAL_PRINTF("                                                                                    \r\n");
            clearFloatArray1();
            clearFloatArray2();
            dataDecoded1 = false;
            dataDecoded2 = false;
            delayMicroseconds(100);         // Make it easier to see DEBUG_PIN on the scope
            digitalWrite(DEBUG_PIN3, HIGH); // debug purposes
        }
    }

    if (millis() > lastSecondMillis + 1000) {
        lastSecondMillis = millis();
        lastLoopCounter = loopCounter;
        loopCounter = 0;
        receiveRate = dataReceived1Counter - lastDataReceivedCounter;
        lastDataReceivedCounter = dataReceived1Counter;
        delay(1);                      // 1mS delay here to make it easier to see DEBUG_PIN on the scope
        digitalWrite(DEBUG_PIN3, LOW); // debug purposes
        #ifndef ARDUINO_ARCH_MBED
            if (!PRINT_DEBUG_DATA) SERIAL_PRINTF("\e[H"); // move to the home position, at the upper left of the screen
            SERIAL_PRINTF("                                                                              \r\n");
        #endif
        SERIAL_PRINTF("                                                                              \r\n");
        SERIAL_PRINTF("Seconds: %07lu  \r\n", lastSecondMillis / 1000);
        SERIAL_PRINTF("LoopRate: %07u  \r\n", lastLoopCounter); // how many loops per second
        SERIAL_PRINTF("receiveRate: %07u  \r\n", receiveRate);  // how many data transmission pairs per second
        SERIAL_PRINTF("drCount: %07u  \r\n", dataReceivedCounter);  // total transmissions received
        SERIAL_PRINTF("dr1Count: %07u dr1SuccessCount: %07u dr1FailureCount: %07u  \r\n", dataReceived1Counter, dataReceived1SuccessCount, dataReceived1FailureCount);
        SERIAL_PRINTF("dr2Count: %07u dr2SuccessCount: %07u dr2FailureCount: %07u  \r\n", dataReceived2Counter, dataReceived2SuccessCount, dataReceived2FailureCount);
        SERIAL_PRINTF("dr1FailureRate: %11.7f percent  \r\n", 100.0f * dataReceived1FailureCount / (dataReceived1Counter > 0 ? dataReceived1Counter : 1));
        SERIAL_PRINTF("dr2FailureRate: %11.7f percent  \r\n", 100.0f * dataReceived2FailureCount / (dataReceived2Counter > 0 ? dataReceived2Counter : 1));
        SERIAL_PRINTF("                                                                              \r\n");
        digitalWrite(DEBUG_PIN3, HIGH); // debug purposes
    }
}
