
#include <Arduino.h>
#include <Wire.h>

// Debug Signal outputs
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)

//#define I2C_BAUDRATE 400000 // Set Clock to 400KHz (Fast Mode) (with 1k pullup resistors, actually runs at 365 KHz)
#define I2C_BAUDRATE 1000000 // Set Clock to 1Mhz (Fast Mode Plus) (with 1k pullup resistors, actually runs at 868 KHz)
#define SLAVE_PICO_ADDRESS 0x30
#define I2C_SDA (6u)
#define I2C_SCL (7u)


#define PRINT_DEBUG_DATA false

bool ledState = false;
PinStatus debugPinInitialState = HIGH;
unsigned int sendCounter = 0, lastSendCount = 0, sendRate = 0;

static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
#define TEST_DATA1_SIZE 21u  // 21 floats will need 21 * 4 = 84 bytes to be sent
float testData1[TEST_DATA1_SIZE] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
#define TEST_DATA2_SIZE 12u  // 12 floats will need 12 * 4 = 48 bytes to be sent
float testData2[TEST_DATA2_SIZE] = {3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2};


#define PRINT_BIN(Num) for (uint32_t t = (1ULL << ((sizeof(Num)*8)-1)); t; t >>= 1) Serial.write(Num  & t ? '1' : '0'); // Prints a binary number with leading zeros
// For checking against scope traces, The floats are stored in memory Least Significant Bytes first
void printFloatArrayAsBinary(float *floatArrPointer, unsigned int size) {
    for (int i = 0; i < size; i++) {
      byte* b1 = ((byte *)floatArrPointer) + (i * 4); // Least Significant Byte
      PRINT_BIN(*b1);Serial.print("-");
      byte* b2 = ((byte *)floatArrPointer)+ (i * 4) + 1;
      PRINT_BIN(*b2);Serial.print("-");
      byte* b3 = ((byte *)floatArrPointer)+ (i * 4) + 2;
      PRINT_BIN(*b3);Serial.print("-");
      byte* b4 = ((byte *)floatArrPointer)+ (i * 4) + 3; // Most Significant Byte
      PRINT_BIN(*b4);Serial.print(","); // use comma to separate each float
    }
}
// For checking against scope protocol decoder, prints the hex values, Least Significant Bytes first
void printFloatArrayAsHex(float *floatArrPointer, unsigned int size) {
    for (int i = 0; i < size; i++) {
      byte* b1 = ((byte *)floatArrPointer) + (i * 4); // Least Significant Byte
      Serial.printf("%02X-", *b1);
      byte* b2 = ((byte *)floatArrPointer)+ (i * 4) + 1;
      Serial.printf("%02X-", *b2);
      byte* b3 = ((byte *)floatArrPointer)+ (i * 4) + 2;
      Serial.printf("%02X-", *b3);
      byte* b4 = ((byte *)floatArrPointer)+ (i * 4) + 3; // Most Significant Byte
      Serial.printf("%02X,", *b4); // use comma to separate each float
    }
}

void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    // Wait for Serial Comms or Timeout after 5 seconds
    while (!Serial && millis() < 5000);
    int startupDelay = 6;
    for (int i = 1; i <= startupDelay; ++i) {
        Serial.printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    Serial.printf("\e[2J\e[H"); // clear screen and go to home position
    Serial.printf("rp2040_chip_version: %d \r\n", rp2040_chip_version());
    Serial.printf("rp2040_rom_version: %d \r\n", rp2040_rom_version());
    Serial.printf("get_core_num: %u \r\n\r\n", get_core_num());

    pinMode(LED_BUILTIN, OUTPUT);

    #ifdef ARDUINO_RASPBERRY_PI_PICO
        //pinMode(23, OUTPUT);
        //digitalWrite(23, HIGH); // Set the SMPS Power Save pin high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #endif

    // Be sure to use pins labeled I2C0 for Wire and I2C1 for Wire1 on the pinout diagram for your board, otherwise it won’t work.
    // For Wire (I2C0 on the Pico) default pins: PIN_WIRE0_SDA (4u) PIN_WIRE0_SCL  (5u)
    // For Wire1 (I2C1 on the Pico) default pins: PIN_WIRE1_SDA (26u), PIN_WIRE1_SCL (27u)
    // Change these pins before calling Wire1.begin() or Wire1.begin()
    #ifdef ARDUINO_ARCH_MBED
        // Arduino MBED core does not support setting the i2c pins like this
    #else
        Wire1.setSDA(I2C_SDA);
        Wire1.setSCL(I2C_SCL);
    #endif
    // Default clock is 100 KHz (with 1k pullup resistors, actually runs at 95.238 KHz)
    Wire1.setClock(I2C_BAUDRATE);
    Wire1.begin();

    // Setup Debug output pins
    pinMode(DEBUG_PIN2, OUTPUT);
    digitalWrite(DEBUG_PIN2, debugPinInitialState);
    pinMode(DEBUG_PIN3, OUTPUT);
    digitalWrite(DEBUG_PIN3, debugPinInitialState);
    pinMode(DEBUG_PIN4, OUTPUT);
    digitalWrite(DEBUG_PIN4, debugPinInitialState);
    pinMode(DEBUG_PIN5, OUTPUT);
    digitalWrite(DEBUG_PIN5, debugPinInitialState);

    Serial.println("Pi Pico i2c test data sender:\r\n");
    Serial.printf("Test Data1: %u floats = %u bytes\r\n", TEST_DATA1_SIZE, TEST_DATA1_SIZE * 4);
    for (int i = 0; i < TEST_DATA1_SIZE; i++) {
        Serial.printf("%5.3f,", testData1[i]);
    }
    Serial.printf("\r\nTest Data1: HEX representations:\r\n");
    printFloatArrayAsHex(testData1, TEST_DATA1_SIZE);
    Serial.printf("\r\nTest Data1: Binary representations:\r\n");
    printFloatArrayAsBinary(testData1, TEST_DATA1_SIZE);
    Serial.print("\r\n\r\n");

    Serial.printf("Test Data2: %u floats = %u bytes\r\n", TEST_DATA2_SIZE, TEST_DATA2_SIZE * 4);
    for (int i = 0; i < TEST_DATA2_SIZE; i++) {
        Serial.printf("%5.3f,", testData2[i]);
    }
    Serial.printf("\r\nTest Data2: HEX representations:\r\n");
    printFloatArrayAsHex(testData2, TEST_DATA2_SIZE);
    Serial.printf("\r\nTest Data2: Binary representations:\r\n");
    printFloatArrayAsBinary(testData2, TEST_DATA2_SIZE);
    Serial.println();
    delay(1000);
}

// Return values:
// 0: success.
// 1: data too long to fit in transmit buffer.
// 2: received NACK on transmit of address.
// 3: received NACK on transmit of data.
// 4: other error.
// 5: timeout
uint8_t sendData(float *floatArrPointer, unsigned int size) {
    Wire1.beginTransmission(SLAVE_PICO_ADDRESS);
    // The Write function only works if an ACK was received from the transmission of the address by beginTransmission(address)
    for (int i = 0; i < size; i++) {
        Wire1.write((byte *)(floatArrPointer + i), 4);
    }
    // Transmissions are buffered (up to 128 bytes) and only performed on endTransmission, as is standard with modern Arduino Wire implementations.
    return Wire1.endTransmission();
}

void sendTestData() {
    sendCounter++;
    // Send the test data
    unsigned long start1 = micros();
    digitalWrite(DEBUG_PIN2, LOW); // signal the start of the send
    uint8_t result1 = sendData(testData1, TEST_DATA1_SIZE);
    digitalWrite(DEBUG_PIN2, HIGH); // signal the end of the send
    unsigned long duration1 = micros() - start1;

    //Experiment with this value to change the failure rate
    delayMicroseconds(1000); // There seems to be a bug in the i2c implementation, no delay here or a delay of less than 400-500 uS causes first byte to go missing at the receiver on next send

    unsigned long start2 = micros();
    digitalWrite(DEBUG_PIN2, LOW); // signal the start of the send
    uint8_t result2 = sendData(testData2, TEST_DATA2_SIZE);
    digitalWrite(DEBUG_PIN2, HIGH); // signal the end of the send
    unsigned long duration2 = micros() - start2;

    bool success1 = false;
    if (result1 == 0) {
        success1 = true;
    } else {
        Serial.printf("result1: I2C Send Error: %u\r\n", result1);
    }

    bool success2 = false;
    if (result2 == 0) {
        success2 = true;
    } else {
        Serial.printf("result2: I2C Send Error: %u\r\n", result2);
    }

    if (PRINT_DEBUG_DATA)  Serial.printf("result1: %s duration1: %lu uS\r\n", success1 ? "true" : "false", duration1);
    if (PRINT_DEBUG_DATA)  Serial.printf("result2: %s duration2: %lu uS\r\n", success2 ? "true" : "false", duration2);
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned long lastHalfSecondMillis = 0;
    static unsigned long lastTenthSecondMillis = 0;
    static unsigned long lastFiftiethSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;

    // // Send data 50 times per second
    // if (millis() > lastFiftiethSecondMillis + 20) {
    //   lastFiftiethSecondMillis = millis();
    //   digitalWrite(LED_BUILTIN, ledState);
    //   ledState = !ledState;
    //   sendTestData();
    // }

    // Send data 10 times per second
    if (millis() > lastTenthSecondMillis + 100) {
      lastTenthSecondMillis = millis();
      digitalWrite(LED_BUILTIN, ledState);
      ledState = !ledState;
      sendTestData();
    }

    // // Send data twice per second
    // if (millis() > lastHalfSecondMillis + 500) {
    //   lastHalfSecondMillis = millis();
    //   digitalWrite(LED_BUILTIN, ledState);
    //   ledState = !ledState;
    //   sendTestData();
    // }

    if (millis() > lastSecondMillis + 1000) {
        lastSecondMillis = millis();
        lastLoopCounter = loopCounter;
        loopCounter = 0;
        sendRate = sendCounter - lastSendCount;
        lastSendCount = sendCounter;
        if (!PRINT_DEBUG_DATA) Serial.printf("\e[H"); // move to the home position, at the upper left of the screen
        Serial.printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\nSeconds: %07lu\r\n", lastSecondMillis / 1000);
        Serial.printf("LoopRate: %07d\r\n", lastLoopCounter); // how many loops per second
        Serial.printf("sendRate: %07d\r\n\r\n", sendRate); // how many loops per second

        // Send data every Second
        // digitalWrite(LED_BUILTIN, ledState);
        // ledState = !ledState;
        // sendTestData();
    }
}
