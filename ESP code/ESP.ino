#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library for interfacing with the LCD display
#include <Wire.h>              // Include the Wire library for I2C communication

// --- ESP32 Pin Definitions ---
#define BUZZER_PIN 22          // Buzzer connected to ESP32 D22 (GPIO22)
#define led 23                 // LED connected to ESP32 D23 (GPIO23)

// Ultrasonic Sensor
#define trigPin 21             // Ultrasonic trigger pin connected to ESP32 D21 (GPIO21)
#define echoPin 19             // Ultrasonic echo pin connected to ESP32 D19 (GPIO19)

// Analog Sensors (using GPIO numbers for analogRead)
// Note: VP (GPIO36), VN (GPIO39) if using ESP32 DevKitC or similar.
// D34 (GPIO34), D35 (GPIO35) are also ADC pins.
#define FLAME_SENSOR_PIN 34    // Flame sensor A0 to ESP32 D34 (GPIO34)
#define MQ2_GAS_SENSOR_PIN 36  // MQ2 gas A0 to ESP32 VP (GPIO36)
#define VIB_SENSOR_PIN 39      // Vib sensor to ESP32 VN (GPIO39)
#define LM35_TEMP_SENSOR_PIN 35 // LM 35 to ESP32 D35 (GPIO35)

// LCD I2C Pins
#define LCD_SDA_PIN 25         // LCD SDA to ESP32 D25 (GPIO25)
#define LCD_SCL_PIN 26         // LCD SCL to ESP32 D26 (GPIO26)

// Define sensor readings for better understanding
// FLAME_SENSOR_PIN = Flame Reading
// MQ2_GAS_SENSOR_PIN = MQ2 Reading
// VIB_SENSOR_PIN = Vibration Reading
// LM35_TEMP_SENSOR_PIN = Temperature Reading

// Initialize LCD with I2C address 0x27, 16 columns, 2 rows, and custom SDA/SCL pins for ESP32
LiquidCrystal_I2C lcd(0x27, 16, 2);

int counter = 0;      // Initialize a variable to store the count of individuals
int inside = 0;       // Initialize a variable to store the count of individuals inside
int outside = 0;      // Initialize a variable to store the count of individuals outside
float analog_temp_raw; // Variable to store the raw analog temperature reading
float temp;           // Variable to store the calculated temperature in Celsius

// Variable to store the last time the counter was updated, to prevent rapid changes
unsigned long lastCountUpdateTime = 0;
const long countUpdateInterval = 500; // Minimum 500ms between counter updates

void setup()
{
    // Set digital pins as outputs
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(led, OUTPUT);

    // Initialize serial communication for debugging and data transmission
    Serial.begin(9600);
    delay(100); // Small delay to ensure Serial is ready

    // Initialize I2C communication for the LCD with specified SDA and SCL pins
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN); // ESP32 specific I2C pin initialization

    delay(1000); // Wait for 1 second for components to stabilize

    // Initialize the LCD display
    lcd.begin(); // Use lcd.begin() for ESP32 LiquidCrystal_I2C
    lcd.clear();       // Clear any previous content
    lcd.backlight();   // Turn on the LCD backlight

    // Set ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    Serial.println("ESP32 Setup Complete. Starting loop...");
}


// Function to measure distance using the ultrasonic sensor, update person counts, and display on LCD
void measureDistanceAndUpdateLCD()
{
    long duration;    // Variable to store pulse duration
    float distance;   // Using float for distance for more precision

    // Static variables to track the last state of the sensor to detect transitions
    static bool objectDetectedClose = false; // True if an object was detected in the "entering" zone
    static bool objectDetectedFar = false;   // True if an object was detected in the "exiting" zone

    // Clear the trigPin to ensure a clean pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Send a 10 microsecond pulse to trigger the ultrasonic sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the pulse on the echoPin (time for sound to travel and return)
    duration = pulseIn(echoPin, HIGH, 30000); // Added timeout for pulseIn (30ms)
    
    // Calculate distance in cm (speed of sound ~0.0343 cm/µs, divided by 2 for round trip)
    // If duration is 0 (timeout), distance will be very large. Handle this.
    if (duration > 0) {
        distance = (duration / 2.0) / 29.1;
    } else {
        distance = 999.0; // Indicate no object detected within timeout range
    }

    // Implement a simple debouncing or cooldown for counter updates
    unsigned long currentMillis = millis();

    // Only allow counting if the sensor provides valid distance and cooldown is over
    if (distance != 999.0 && currentMillis - lastCountUpdateTime >= countUpdateInterval) {
        // Original logic thresholds: <= 9 for inside, > 9 and <= 18 for outside.
        // Let's re-implement this with state-based debouncing to prevent multiple counts.

        if (distance <= 9.0) { // Person is in the 'entering' zone (e.g., 0-9cm)
            if (!objectDetectedClose) { // If not already in 'entering' state
                counter++;
                inside++;
                Serial.println("-> Detected Entrance. Incrementing. Total: " + String(counter));
                objectDetectedClose = true;
                objectDetectedFar = false; // Reset far state if close detected
                lastCountUpdateTime = currentMillis;
            }
        } else if (distance > 9.0 && distance <= 18.0) { // Person is in the 'exiting' zone (e.g., 9-18cm)
            if (!objectDetectedFar) { // If not already in 'exiting' state
                if (counter > 0) { // Only decrement if there are people currently inside
                    counter--;
                    outside++;
                    Serial.println("<- Detected Exit. Decrementing. Total: " + String(counter));
                    objectDetectedFar = true;
                    objectDetectedClose = false; // Reset close state if far detected
                    lastCountUpdateTime = currentMillis;
                } else {
                     Serial.println("<- Detected Exit, but counter is already 0.");
                }
            }
        } else { // No object in counting zones, or too far (e.g., >18cm or <0cm which won't happen)
            objectDetectedClose = false;
            objectDetectedFar = false;
            // No counter update
        }
    } else if (distance == 999.0) { // If ultrasonic sensor is not detecting anything valid
        // Reset state if no valid object is detected for a prolonged period
        objectDetectedClose = false;
        objectDetectedFar = false;
    }

    // Display updated counts on the LCD (always display current values)
    // Clear previous values to avoid ghosting (e.g., 10 becoming 1 and leaving the '0')
    lcd.setCursor(4, 0);
    lcd.print("   "); 
    lcd.setCursor(4, 0);
    lcd.print(inside);

    lcd.setCursor(13, 0);
    lcd.print("   "); 
    lcd.setCursor(13, 0);
    lcd.print(outside);

    lcd.setCursor(14, 1);
    lcd.print("   "); 
    lcd.setCursor(14, 1);
    lcd.print(counter);
}

void loop()
{
    // Read analog sensor values
    analog_temp_raw = analogRead(LM35_TEMP_SENSOR_PIN);           // Read raw analog value from LM35
    temp = ((analog_temp_raw * 500.0) / 4095.0) - 5.0; // Convert raw 12-bit analog (0-4095) to Celsius
                                                      // ESP32 ADC is 12-bit (0-4095), assuming 5V reference.
                                                      // Adjust 500.0 to your actual reference voltage if needed.
    int gas = analogRead(MQ2_GAS_SENSOR_PIN);                   // Read MQ2 sensor value
    int FlameValueRaw = analogRead(FLAME_SENSOR_PIN);            // Read raw flame sensor value

    // Map FlameValueRaw (4095 to 3100) to FlameValue (0 to 1500)
    // 4095 (no flame) -> 0 (no danger)
    // 3100 (strong flame) -> 1500 (high danger)
    int FlameValue = map(FlameValueRaw, 4095, 3100, 0, 1500);
    // Clamp the value to ensure it stays within 0-1500 range
    FlameValue = constrain(FlameValue, 0, 1500);

    int vib = analogRead(VIB_SENSOR_PIN);                       // Read vibration sensor value

    // Check for potential threats based on sensor readings
    // Flame alert condition changed from < 150 to > 200
    if (gas > 300 || FlameValue > 500 || vib > 300 || temp < 0) 
    {
        digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
        digitalWrite(led, LOW);         // Turn on LED for alert indication
        lcd.clear();                    // Clear LCD before displaying alert
        delay(100); // Short delay for stability before printing alert

        // Display specific alert messages on LCD and Serial Monitor
        if (gas > 250)
        {
            lcd.setCursor(0, 0);
            lcd.print("SMOKE DETECTED");
            Serial.println("ALERT: SMOKE DETECTED!");
        }
        // Flame alert condition changed from < 150 to > 200
        if (FlameValue > 500) 
        {
            lcd.setCursor(0, 0);
            lcd.print("FIRE DETECTED");
            Serial.println("ALERT: FIRE DETECTED! Value: " + String(FlameValue)); // Added FlameValue to alert
        }
        if (vib > 300)
        {
            lcd.setCursor(0, 0);
            lcd.print("Vibration Alert");
            // Serial.println("ALERT: Vibration DETECTED! Value: " + String(vib)); // Added vib value to alert
        }
        if (temp < 0) // Example: if temperature drops below 0, it's an alert
        {
            lcd.setCursor(0, 0);
            lcd.print("Temp. Alert");
            Serial.println("ALERT: Abnormally Low Temperature DETECTED!");
        }

        // Display total inside count during alert
        lcd.setCursor(0, 1);
        lcd.print("Total Inside: ");
        lcd.setCursor(14, 1);
        lcd.print(counter);

        delay(500); // Keep alert message visible for 2 seconds
        lcd.clear(); // Clear LCD after alert
    }
    else // No active threats, proceed with normal operation and update LCD
    {
        digitalWrite(led, HIGH);     // Turn on LED (assuming HIGH means 'safe' or 'normal operation')
        digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer (using LOW for off)

        // Clear and update LCD with current status
        lcd.clear();
        lcd.print("IN: ");
        lcd.setCursor(8, 0);
        lcd.print("Out: ");
        lcd.setCursor(0, 1);
        lcd.print("Total Inside: ");

        measureDistanceAndUpdateLCD(); // Update distance and people counts
    }
    
    // Construct the string in the EXACT format expected by the Next.js API's regex
    // This is moved outside the if/else block to ALWAYS send sensor data to the API.
    String outputString = "MQ2: " + String(gas) +
                          " | Vibration: " + String(vib) +
                          " | Flame: " + String(FlameValue) + // Use the mapped flame value
                          " | Temp: " + String(temp, 1) + // Format temp to one decimal place
                          " | In: " + String(inside) +
                          " | Out: " + String(outside) +
                          " | Total: " + String(counter);
    Serial.println(outputString); // Send the formatted string to serial for Next.js API

    delay(200); // Delay for stability between loop iterations
}
