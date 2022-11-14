
#include <Arduino.h>
#include <Wire.h>

// Debug Signal outputs, to allow us to observe any error conditions on the scope
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)

#define SLAVE_PICO_ADDRESS 0x30
// #define I2C_SDA (6u)
// #define I2C_SCL (7u)
#define I2C_SDA (26u)
#define I2C_SCL (27u)

#define PRINT_DEBUG_DATA false
#define SERIAL_DURING_I2C true

bool ledState = false;
PinStatus debugPinInitialState = HIGH;

volatile unsigned int  dataReceivedCounter = 0, dataReceived1Counter = 0, dataReceived1Bytes = 0, dataReceived2Counter = 0, dataReceived2Bytes = 0;
volatile bool dataReceived1 = false, dataReceived2 = false, allDataReceived = false;
unsigned int receiveRate = 0, lastDataReceivedCounter = 0;
unsigned int dataReceived1SuccessCount = 0, dataReceived2SuccessCount = 0;
unsigned int dataReceived1FailureCount = 0, dataReceived2FailureCount = 0;
bool dataNotAvailable1 = false, dataNotAvailable2 = false;
bool dataDecoded1 = false, dataDecoded2 = false;

// Code from the transmitter sketch, used for verification
#define TEST_DATA1_SIZE 21u  // 21 floats will need 21 * 4 = 84 bytes to be sent
float testData1[TEST_DATA1_SIZE] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
#define TEST_DATA2_SIZE 12u  // 12 floats will need 12 * 4 = 48 bytes to be sent
float testData2[TEST_DATA2_SIZE] = {3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2};

// The transmit buffer is 128 bytes, not sure that the receive buffer size is??
#define RECV_BUFF_MAX_SIZE (byte)128
#define DATA1_SIZE (84u) // use the same size as originally encountered
#define DATA2_SIZE (48u) // use the same size as originally encountered
byte receiveBuff1[DATA1_SIZE]; // I2C Receive buffer for first transmission
byte receiveBuff2[DATA2_SIZE]; // I2C Receive buffer for second transmission
static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
float decodedFloatValues1[DATA1_SIZE / 4]; // 21 floats
float decodedFloatValues2[DATA2_SIZE / 4]; // 12 floats

// Called when the I2C slave gets written to
// This code takes 36 uS for 84 bytes, 21 uS for 48 bytes
// delay from end of transmission from master to interrupt firing is about 230 uS
// Interval between 2 consecutive sends from the master is 1000uS
void recv(int byteCount) {
    digitalWrite(DEBUG_PIN2, LOW); // signal the start of the interrupt
    digitalWrite(LED_BUILTIN, HIGH);
    dataReceivedCounter++;
    if (byteCount > RECV_BUFF_MAX_SIZE) byteCount = RECV_BUFF_MAX_SIZE;
    int i;
    if (byteCount > DATA2_SIZE) { // New checking method for data1, as data1 is somewhat larger than data2, allows capture if less than expected size
        allDataReceived = false; // Data1 arrives first
        dataReceived1Counter++;
        dataReceived1Bytes = byteCount;
        for (i = 0; i < DATA1_SIZE; i++) { // UPDATED: now reads the expected number of bytes instead of byteCount
            int data = Wire1.read(); // returns -1 if no ddata is available
            if (data > -1) {
                receiveBuff1[i] = data; // read into data1 buffer
            } else {
                dataNotAvailable1 = true;
            }
        }
        dataReceived1 = true;
    } else if (byteCount <= DATA2_SIZE) { // also allows capture if less than expected size
        dataReceived2Counter++;
        dataReceived2Bytes = byteCount;
        for (i = 0; i < DATA2_SIZE; i++) { // UPDATED: now reads the expected number of bytes instead of byteCount
            int data = Wire1.read(); // returns -1 if no ddata is available
            if (data > -1) {
               receiveBuff2[i] = data; // read into data2 buffer
            } else {
                dataNotAvailable2 = true;
            }
        }
        dataReceived2 = true;
        allDataReceived = true;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_PIN2, HIGH); // signal the end of the interrupt
}

// Called when the I2C slave is read from
void req() {
}

void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    // Wait for Serial Comms or Timeout after 5 seconds
    while (!Serial && millis() < 5000);

    pinMode(LED_BUILTIN, OUTPUT);

    // Be sure to use pins labeled I2C0 for Wire and I2C1 for Wire1 on the pinout diagram for your board, otherwise it won’t work.
    // For Wire (I2C0 on the Pico) default pins: PIN_WIRE0_SDA (4u) PIN_WIRE0_SCL  (5u)
    // For Wire1 (I2C1 on the Pico) default pins: PIN_WIRE1_SDA (26u), PIN_WIRE1_SCL (27u)
    // Change these pins before calling Wire1.begin() or Wire1.begin()
    Wire1.setSDA(I2C_SDA);
    Wire1.setSCL(I2C_SCL);
    // Default clock is 100 KHz (with 1k pullup resistors, actually runs at 95.238 KHz)
    Wire1.setClock(400000); // Set Clock to 400KHz (Fast Mode) (with 1k pullup resistors, actually runs at 365 KHz)
    // Wire1.setClock(1000000); // Set Clock to 1Mhz (Fast Mode Plus) (with 1k pullup resistors, actually runs at 868 KHz)
    Wire1.begin(SLAVE_PICO_ADDRESS); // Join the i2c bus as slave
    Wire1.onReceive(recv);           // i2c receive interrupt
    Wire1.onRequest(req);            // i2c request (send) interrupt

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
    Serial.println("Pi Pico i2c receiver:");
    Serial.println("Ready to Receive Data...");
    if (dataReceivedCounter > 0) {
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
  for (int i = 0; i < DATA1_SIZE; i++) receiveBuff1[i] = 0;
}

void clearReceiveBuffer2() {
  for (int i = 0; i < DATA2_SIZE; i++) receiveBuff2[i] = 0;
}

void clearFloatArray1() {
  for (int i = 0; i < DATA1_SIZE / 4; i++) decodedFloatValues1[i] = 0;
}

void clearFloatArray2() {
  for (int i = 0; i < DATA2_SIZE / 4; i++) decodedFloatValues2[i] = 0;
}

void decodeData(byte *byteArrPointer, unsigned int dataReceivedBytes, float *floatArrayPointer) {
    int i;
    if (PRINT_DEBUG_DATA) Serial.printf("Received Bytes: %d \r\n", dataReceivedBytes);
    for (i = 0; i < dataReceivedBytes; i += 4) {
        uint8_t byte = *(byteArrPointer + i);
        float f;
        memcpy(&f, byteArrPointer + i, 4);
        if (PRINT_DEBUG_DATA) Serial.printf("f%d: %+8.3f \r\n", i / 4, f);
        *(floatArrayPointer + (i / 4)) = f;
    }
    if (PRINT_DEBUG_DATA) Serial.printf("end: %d \r\n\r\n", i);
}

void verifyData1(int counter) {
    Serial.print("verifying decodedFloatValues1:                                                             \r\n");
    for (int i = 0; i < TEST_DATA1_SIZE; i++) {
        if (testData1[i] != decodedFloatValues1[i]) {
            Serial.printf("                                                      counter:%d floatValues1 Error! Index: %d expectedFloat: %3.1f receivedFloat: %3.1f \r\n", counter, i, testData1[i], decodedFloatValues1[i]);
        }
    }
    //Serial.print("                                                                                                         \r\n");
}

void verifyData2(int counter) {
    Serial.print("verifying decodedFloatValues2:                                                             \r\n");
    for (int i = 0; i < TEST_DATA2_SIZE; i++) {
        if (testData2[i] != decodedFloatValues2[i]) {
            Serial.printf("                                                      counter:%d floatValues2 Error! Index: %d expectedFloat: %3.1f receivedFloat: %3.1f \r\n", counter, i, testData2[i], decodedFloatValues2[i]);
        }
    }
    //Serial.print("                                                                                                         \r\n");
}

void printData() {
    Serial.print("decodedFloatValues1:                                                        \r\n");
    for (int i = 0; i < DATA1_SIZE / 4; i++) {
        Serial.printf("%3.1f,", decodedFloatValues1[i]);
    }
    Serial.print("\r\ndecodedFloatValues2:                                                        \r\n");
    for (int i = 0; i < DATA2_SIZE / 4; i++) {
        Serial.printf("%3.1f,", decodedFloatValues2[i]);
    }
    Serial.print("\r\n                                                                            \r\n");
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;
    if (loopCounter > 1) {
        if (dataReceived1 || dataReceived2) {
            digitalWrite(DEBUG_PIN3, LOW); // signal the start of the work for debug purposes
            if (dataReceived1) {
                if (dataReceived1Bytes == DATA1_SIZE) {
                    dataReceived1SuccessCount++;
                    if (SERIAL_DURING_I2C) Serial.printf("dr1: Success, received: %u bytes                 | ", dataReceived1Bytes);
                } else {
                    dataReceived1FailureCount++;
                    digitalWrite(DEBUG_PIN4, LOW); // debug purposes
                    if (SERIAL_DURING_I2C) Serial.printf("dr1: Error!!, received: %u expected: %u bytes! | ", dataReceived1Bytes, DATA1_SIZE);
                    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
                    digitalWrite(DEBUG_PIN4, HIGH); // debug purposes
                }
                if (dataNotAvailable1) {
                    if (SERIAL_DURING_I2C) Serial.print("                                                   dataNotAvailable1: wire.read() returned no data! | ");
                }
                decodeData(receiveBuff1, dataReceived1Bytes, decodedFloatValues1);
                dataDecoded1 = true;
                clearReceiveBuffer1();
                dataReceived1 = false; // Reset the Data Received flag after it has been collected
                dataNotAvailable1 = false;
            }
            if (dataReceived2 && !dataReceived1) {
                if (dataReceived2Bytes == DATA2_SIZE) {
                    dataReceived2SuccessCount++;
                    if (SERIAL_DURING_I2C) Serial.printf("dr2: Success, received: %u bytes                 ", dataReceived2Bytes);
                } else {
                    dataReceived2FailureCount++;
                    digitalWrite(DEBUG_PIN4, LOW); // debug purposes
                    if (SERIAL_DURING_I2C) Serial.printf("dr2: Error!!, received: %u expected: %u bytes! ", dataReceived2Bytes, DATA2_SIZE);
                    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
                    digitalWrite(DEBUG_PIN4, HIGH); // debug purposes
                }
                if (dataNotAvailable2) {
                    if (SERIAL_DURING_I2C) Serial.print("   dataNotAvailable2: wire.read returned no data ");
                }
                Serial.println();
                decodeData(receiveBuff2, dataReceived2Bytes, decodedFloatValues2);
                dataDecoded2 = true;
                clearReceiveBuffer2();
                dataReceived2 = false; // Reset the Data Received flag after it has been collected
                dataNotAvailable2 = false;
            }
            digitalWrite(DEBUG_PIN3, HIGH); // signal the end of the work for debug purposes
        }

        if (allDataReceived && dataDecoded1 && dataDecoded2) {
            delay(1);                      // 1mS delay here to make it easier to see DEBUG_PIN on the scope
            digitalWrite(DEBUG_PIN3, LOW); // debug purposes
            if (dataReceived1Bytes == DATA1_SIZE) {
                if (!SERIAL_DURING_I2C) Serial.printf("dr1: Success, received: %u bytes                 | ", dataReceived1Bytes);
            } else {
                if (!SERIAL_DURING_I2C) Serial.printf("dr1: Error!!, received: %u expected: %u bytes! | ", dataReceived1Bytes, DATA1_SIZE);
            }
            if (dataReceived2Bytes == DATA2_SIZE) {
                if (!SERIAL_DURING_I2C) Serial.printf("dr2: Success, received: %u bytes                 \r\n", dataReceived2Bytes);
            } else {
                if (!SERIAL_DURING_I2C) Serial.printf("dr2: Error!!, received: %u expected: %u bytes! \r\n", dataReceived2Bytes, DATA2_SIZE);
            }
            verifyData1(dataReceived1Counter);
            verifyData2(dataReceived2Counter);
            if (PRINT_DEBUG_DATA) printData();
            //printData();
            Serial.print("                                                                                    \r\n");
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
        Serial.print("                                                                              \r\n");
        Serial.print("                                                                              \r\n");
        if (!PRINT_DEBUG_DATA) Serial.printf("\e[H"); // move to the home position, at the upper left of the screen
        Serial.print("                                                                              \r\n");
        Serial.print("                                                                              \r\n");
        Serial.printf("Seconds: %07u  \r\n", lastSecondMillis / 1000);
        Serial.printf("LoopRate: %07u  \r\n", lastLoopCounter); // how many loops per second
        Serial.printf("receiveRate: %07u  \r\n", receiveRate);  // how many data transmission pairs per second
        Serial.printf("drCount: %07u  \r\n", dataReceivedCounter);  // total transmissions received
        Serial.printf("dr1Count: %07u dr1SuccessCount: %07u dr1FailureCount: %07u  \r\n", dataReceived1Counter, dataReceived1SuccessCount, dataReceived1FailureCount);
        Serial.printf("dr2Count: %07u dr2SuccessCount: %07u dr2FailureCount: %07u  \r\n", dataReceived2Counter, dataReceived2SuccessCount, dataReceived2FailureCount);
        Serial.printf("dr1FailureRate: %11.7f percent  \r\n", 100.0f * dataReceived1FailureCount / (dataReceived1Counter > 0 ? dataReceived1Counter : 1));
        Serial.printf("dr2FailureRate: %11.7f percent  \r\n", 100.0f * dataReceived2FailureCount / (dataReceived2Counter > 0 ? dataReceived2Counter : 1));
        Serial.print("\r\n                                                                              \r\n");
        digitalWrite(DEBUG_PIN3, HIGH); // debug purposes
    }
}