#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>    // Include the SoftwareSerial library for serial communication with SIM800L module

SoftwareSerial sim(8, 7); // Create a SoftwareSerial object named 'sim' with pins tx- 8 and rx- 7 for communication with SIM800L module

// LCD setup (Address: 0x27 or 0x3F depending on the module)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin Definitions
#define LED_PIN 6
#define BUZZER_PIN 5
#define TRIG_PIN 12
#define ECHO_PIN 13
#define MQ2_PIN A0
#define VIBRATION_PIN A1
#define FLAME_PIN A2
#define LM35_PIN A3

// Thresholds for hazard detection
#define MQ2_THRESHOLD 400
#define VIBRATION_THRESHOLD 200
#define FLAME_THRESHOLD 550
#define LM35_THRESHOLD 60

// Ultrasonic variables
int inCount = 0, outCount = 0, totalInside = 0;

// Buzzer control variables
bool buzzerOn = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION = 2000;  // 2 seconds

//for calling
int _timeout;                    // Initialize a variable for timeout
String _buffer;                  // Initialize a string buffer
String number = "+917499440115"; // Define an emergency contact number

unsigned long lastDetectionTime = 0;
const unsigned long DETECTION_DELAY = 1000; // 1 second

// Function to get distance from ultrasonic sensor
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2;  // Convert to cm
    return distance;
}

void setup() {
    Serial.begin(9600);
    _buffer.reserve(50);         // Reserve memory for the string buffer
    sim.begin(9600);             // Initialize serial communication with SIM800L module
    delay(1000);                 // Wait for 1 second

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(MQ2_PIN, INPUT);
    pinMode(VIBRATION_PIN, INPUT);
    pinMode(FLAME_PIN, INPUT);
    pinMode(LM35_PIN, INPUT);

    digitalWrite(LED_PIN, HIGH);  // LED normally ON
    digitalWrite(BUZZER_PIN, LOW); // Buzzer OFF

    lcd.init();
    lcd.backlight();
}

String _readSerial()
{
    _timeout = 0;                                // Reset timeout counter
    while (!sim.available() && _timeout < 12000) // Wait for SIM module to be available or timeout
    {
        delay(13);  // Delay for stability
        _timeout++; // Increment timeout counter
    }
    if (sim.available())
    {                            // If SIM module is available
        return sim.readString(); // Read string from SIM module
    }
}

void callNumber()
{
    sim.print(F("ATD"));     // Send command to initiate a call
    sim.print(number);       // Send emergency contact number
    sim.print(F(";\r\n"));   // End command
    _buffer = _readSerial(); // Read response from SIM module
    Serial.println("buffer : "); // Print response to serial monitor
    Serial.println(_buffer); // Print response to serial monitor
}

void loop() {
    int mq2Value = analogRead(MQ2_PIN);
    int vibrationValue = analogRead(VIBRATION_PIN);
    int flameValue = analogRead(FLAME_PIN);
    int lm35Value = analogRead(LM35_PIN);

    // Convert LM35 analog value to temperature
    float temperature = (lm35Value * 5.0 / 1023.0) * 100;

    // Check for hazards
    bool hazardDetected = false;
    String hazardMessage = "";

    if (mq2Value > MQ2_THRESHOLD) {
        hazardDetected = true;
        hazardMessage = "Gas Leak!";
        // callNumber();                     // Call emergency number
        // if (sim.available() > 0)
        //     {                             // Check if SIM module is available
        //         Serial.write(sim.read()); // Write response from SIM module to serial monitor
        //     }
    }
    if (vibrationValue > VIBRATION_THRESHOLD) {
        hazardDetected = true;
        hazardMessage = "Vibration!";
    }
    if (flameValue < FLAME_THRESHOLD) {
        hazardDetected = true;
        hazardMessage = "Fire!";
        // callNumber();                     // Call emergency number
        // if (sim.available() > 0)
        //     {                             // Check if SIM module is available
        //         Serial.write(sim.read()); // Write response from SIM module to serial monitor
        //     }
    }
    if (temperature > LM35_THRESHOLD) {
        hazardDetected = true;
        hazardMessage = "Overheat!";
    }

    // Handle hazard response
    if (hazardDetected) {
        digitalWrite(LED_PIN, LOW);
        
        if (!buzzerOn) {  // Start buzzer timer only if it's not already on
            buzzerOn = true;
            buzzerStartTime = millis();
            digitalWrite(BUZZER_PIN, HIGH);
        }
    } else {
        digitalWrite(LED_PIN, HIGH);
    }

    // Turn off buzzer after BUZZER_DURATION
    if (buzzerOn && millis() - buzzerStartTime >= BUZZER_DURATION) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerOn = false;
    }

    // Call ultrasonic function
    trackPeopleMovement();

    // Print values to Serial Monitor
    Serial.print("MQ2: "); Serial.print(mq2Value);
    Serial.print(" | Vibration: "); Serial.print(vibrationValue);
    Serial.print(" | Flame: "); Serial.print(flameValue);
    Serial.print(" | Temp: "); Serial.print(temperature);
    Serial.print(" | In: "); Serial.print(inCount);
    Serial.print(" | Out: "); Serial.print(outCount);
    Serial.print(" | Total: "); Serial.println(totalInside);

    // Update LCD display immediately
    lcd.clear();
    if (hazardDetected) {
        lcd.setCursor(0, 0);
        lcd.print(hazardMessage);
        lcd.setCursor(0, 1);
        lcd.print("Total: "); lcd.print(totalInside);
    } else {
        lcd.setCursor(0, 0);
        lcd.print("In: "); lcd.print(inCount);
        lcd.print(" Out: "); lcd.print(outCount);
        lcd.setCursor(0, 1);
        lcd.print("Total: "); lcd.print(totalInside);
    }

    delay(500);  // Small delay before next iteration
}


void trackPeopleMovement() {
    int distance = getDistance();
    unsigned long currentTime = millis();

    if (currentTime - lastDetectionTime >= DETECTION_DELAY) {
        if (distance > 0 && distance <= 5) {
            inCount++;
            lastDetectionTime = currentTime;
        } else if (distance > 5 && distance <= 10) {
            outCount++;
            lastDetectionTime = currentTime;
        }

        // Ensure totalInside never goes negative
        totalInside = inCount - outCount;
        if (totalInside < 0) {
            totalInside = 0;
            outCount = inCount;  // Adjust outCount to match
        }
    }
}

