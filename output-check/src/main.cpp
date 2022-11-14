
#include <Arduino.h>
#include <Wire.h>

#ifdef ARDUINO_ARCH_MBED
REDIRECT_STDOUT_TO(Serial); // https://github.com/platformio/platform-raspberrypi/issues/24
#endif
#ifdef ARDUINO_ARCH_MBED
#define SERIAL_PRINTF(...) printf(__VA_ARGS__)
#else
#define SERIAL_PRINTF(...) Serial.printf(__VA_ARGS__)
#endif

// Debug Signal outputs, to allow us to observe any error conditions on the scope
#define DEBUG_PIN2 (6u)
#define DEBUG_PIN3 (7u)
#define DEBUG_PIN4 (8u)
#define DEBUG_PIN5 (9u)

#define INTERRUPT_PIN (15u)
volatile unsigned int interruptCounter = 0;
volatile bool interruptFlag = false;

bool ledState = false;
int debugPinInitialState = HIGH;

// Connect DEBUG_PIN3 to INTERRUPT_PIN to fire this ISR
void ISR() {
    interruptCounter++;
    interruptFlag = true;
    digitalWrite(DEBUG_PIN2, LOW);
    delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
    digitalWrite(DEBUG_PIN2, HIGH);
}

void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    // Wait for Serial Comms or Timeout after 5 seconds
    while (!Serial && millis() < 5000)
        ;

    pinMode(LED_BUILTIN, OUTPUT);

    // Setup Interrupt for data ready pin
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), ISR, FALLING);

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
    Serial.println("Output Checker");
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned long lastTenthSecondMillis = 0;
    static unsigned long lastHundredthSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;

    // Runs data 100 times per second
    if (millis() > lastHundredthSecondMillis + 10) {
        lastHundredthSecondMillis = millis();
        digitalWrite(DEBUG_PIN3, LOW);
        delayMicroseconds(10);          // Make it easier to see DEBUG_PIN on the scope
        digitalWrite(DEBUG_PIN3, HIGH);
    }

    // Runs 10 times per second
    if (millis() > lastTenthSecondMillis + 100) {
        lastTenthSecondMillis = millis();
        digitalWrite(LED_BUILTIN, ledState);
        ledState = !ledState;
        digitalWrite(DEBUG_PIN4, LOW);
        delayMicroseconds(100);         // Make it easier to see DEBUG_PIN on the scope
        digitalWrite(DEBUG_PIN4, HIGH);
    }

    // runs every second
    if (millis() > lastSecondMillis + 1000) {
        lastSecondMillis = millis();
        lastLoopCounter = loopCounter;
        loopCounter = 0;
        digitalWrite(DEBUG_PIN5, LOW);
        delayMicroseconds(1000);        // Make it easier to see DEBUG_PIN on the scope
        digitalWrite(DEBUG_PIN5, HIGH);
        SERIAL_PRINTF("\e[H");          // move to the home position, at the upper left of the screen
        SERIAL_PRINTF("\r\n\r\nSeconds: %07lu  \r\n", lastSecondMillis / 1000);
        SERIAL_PRINTF("LoopRate: %07u  \r\n", lastLoopCounter); // how many loops per second
        SERIAL_PRINTF("interruptCounter: %07u  \r\n", interruptCounter);
    }
}
