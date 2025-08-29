#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library for interfacing with the LCD display
#include <Wire.h>              // Include the Wire library for I2C communication


#define BUZZER_PIN 5 // Define the pin for the buzzer
#define led 6        // Define the pin for the LED
#define trigPin 12   // Define the pin for the trigger pin of the distance sensor
#define echoPin 13   // Define the pin for the echo pin of the distance sensor

// Define sensor readings for better understanding
// A0 = MQ2 Reading
// A1 = Flame Reading
// A2 = Vibration Reading
// A3 = Temperature Reading

LiquidCrystal_I2C lcd(0x27, 16, 2); // Create an instance of the LiquidCrystal_I2C class with the I2C address, and LCD dimensions

int counter = 0;                 // Initialize a variable to store the count of individuals
int inside = 0;                  // Initialize a variable to store the count of individuals inside
int outside = 0;                 // Initialize a variable to store the count of individuals outside
float analog_temp;               // Initialize a variable to store the analog temperature reading
float temp;                      // Initialize a variable to store the calculated temperature

void setup()
{
    pinMode(BUZZER_PIN, OUTPUT); // Set BUZZER_PIN as an output
    pinMode(led, OUTPUT);        // Set led as an output
    Serial.begin(9600);          // Initialize serial communication at 9600 baud
    _buffer.reserve(50);         // Reserve memory for the string buffer
    sim.begin(9600);             // Initialize serial communication with SIM800L module
    delay(1000);                 // Wait for 1 second
    lcd.init();                  // Initialize the LCD
    lcd.clear();                 // Clear the LCD display
    lcd.backlight();             // Turn on the backlight of the LCD
    pinMode(trigPin, OUTPUT);    // Set trigPin as an output
    pinMode(echoPin, INPUT);     // Set echoPin as an input
}


// Function to measure distance, update counters, and display on LCD
void measureDistanceAndUpdateLCD()
{
    long duration, distance; // Declare variables to store duration and distance

    digitalWrite(trigPin, LOW);  // Send a LOW signal to the trigger pin
    delayMicroseconds(2);        // Wait for 2 microseconds
    digitalWrite(trigPin, HIGH); // Send a HIGH signal to the trigger pin
    delayMicroseconds(10);       // Wait for 10 microseconds
    digitalWrite(trigPin, LOW);  // Send a LOW signal to the trigger pin

    duration = pulseIn(echoPin, HIGH); // Measure the duration of the pulse from the echo pin
    distance = (duration / 2) / 29.1;  // Calculate the distance based on the duration

    if (distance <= 9)
    {              // Check if distance is less than or equal to 9
        counter++; // Increment counter
        inside++;  // Increment inside count
    }
    else if (distance > 9 && distance <= 18)
    {              // Check if distance is greater than 9 and less than or equal to 18
        counter--; // Decrement counter
        outside++; // Increment outside count
    }

    lcd.setCursor(4, 0);  // Set cursor to first row, fifth column
    lcd.print(inside);    // Display inside count
    lcd.setCursor(13, 0); // Set cursor to first row, fourteenth column
    lcd.print(outside);   // Display outside count
    lcd.setCursor(14, 1); // Set cursor to second row, fifteenth column
    lcd.print(counter);   // Display total count

    if (counter > 9 || counter < 0)
    {                // Check if counter is out of bounds
        delay(100);  // Delay for stability
        lcd.clear(); // Clear LCD display
    }
}

void graph_ready(int gas, int FlameValue, int vib, int temp , int inside , int outside , int counter )
{
    // Print a separator line to distinguish different sets of data
    Serial.println("--------------------------------------------------------");
    // Print the gas reading label
    Serial.println("MQ2 Reading : ");
    // Print the gas reading value
    Serial.println(gas);
    // Print the flame reading label
    Serial.println("Flame Reading : ");
    // Print the flame reading value
    Serial.println(FlameValue);
    // Print the vibration reading label
    Serial.println("Vibration Reading : ");
    // Print the vibration reading value
    Serial.println(vib);
    // Print the temperature reading label
    Serial.println("Temperature Reading : ");
    // Print the temperature reading value
    Serial.println(temp);
    // Print a separator line to distinguish different sets of data
    Serial.println("--------------------------------------------------------");

    // Print a separator line to distinguish between sensor readings and internal variables
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    // Print the label for the count of individuals inside
    Serial.println("IN: ");
    // Print the count of individuals inside
    Serial.println(inside);
    // Print the label for the count of individuals outside
    Serial.println("Out: ");
    // Print the count of individuals outside
    Serial.println(outside);
    // Print the label for the total count of individuals inside
    Serial.println("Total Inside: ");
    // Print the total count of individuals inside
    Serial.println(counter);
    // Print a separator line to distinguish between sensor readings and internal variables
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

void loop()
{
    analog_temp = analogRead(A3);            // Read analog temperature value
    temp = ((analog_temp * 500) / 1023) - 5; // Convert analog temperature value to Celsius
    int gas = analogRead(A0);                // Read MQ2 sensor value
    int FlameValue = analogRead(A2);         // Read flame sensor value
    int vib = analogRead(A1);                // Read vibration sensor value
    int analog_temp = analogRead(A3);        // Read analog temperature value again


    Serial.print(gas);        // Print MQ2 sensor value to serial monitor
    Serial.print(",");        // Print delimiter
    Serial.print(FlameValue); // Print flame sensor value to serial monitor
    Serial.print(",");        // Print delimiter
    Serial.print(vib);        // Print vibration sensor value to serial monitor
    Serial.print(",");        // Print delimiter
    Serial.println(temp);     // Print temperature to serial monitor

    if (gas > 300 || FlameValue < 150 || vib > 300 || temp < 0)
    {                // Check for potential threats
        lcd.clear(); // Clear LCD display
        delay(1000); // Delay for stability
        if (gas > 250)
        {                                     // Check if gas level is high
            digitalWrite(BUZZER_PIN, HIGH);       // Activate buzzer
            digitalWrite(led, LOW);           // Turn on LED
            lcd.setCursor(0, 0);              // Set cursor to first row, first column
            lcd.print("SMOKE DETECTED");      // Display "SMOKE DETECTED" on LCD
            Serial.println("SMOKE DETECTED"); // Print "SMOKE DETECTED" to serial monitor
        }
        if (FlameValue < 150)
        {                                    // Check if flame is detected
            digitalWrite(BUZZER_PIN, HIGH);      // Activate buzzer
            digitalWrite(led, LOW);          // Turn on LED
            lcd.setCursor(0, 0);             // Set cursor to first row, first column
            lcd.print("FIRE DETECTED");      // Display "FIRE DETECTED" on LCD
            Serial.println("FIRE DETECTED"); // Print "FIRE DETECTED" to serial monitor
            callNumber();                    // Call emergency number
        }
        if (vib > 300)
        {                                         // Check if vibration level is high
            digitalWrite(BUZZER_PIN, HIGH);           // Activate buzzer
            digitalWrite(led, LOW);               // Turn on LED
            lcd.setCursor(0, 0);                  // Set cursor to first row, first column
            lcd.print("Vibration Alert");         // Display "Vibration Alert" on LCD
            Serial.println("Vibration DETECTED"); // Print "Vibration DETECTED" to serial monitor
        }
        if (temp < 0)
        {                                                // Check if temperature is high
            digitalWrite(BUZZER_PIN, HIGH);                  // Activate buzzer
            digitalWrite(led, LOW);                      // Turn on LED
            lcd.setCursor(0, 0);                         // Set cursor to first row, first column
            lcd.print("Temp. Alert");                    // Display "Temp. Alert" on LCD
            Serial.println("High Temperature DETECTED"); // Print "High Temperature DETECTED" to serial monitor
        }

        lcd.setCursor(0, 1);              // Set cursor to second row, first column
        lcd.print("Total Inside: ");      // Display "Total Inside:" on LCD
        Serial.println("Total Inside: "); // Print "Total Inside:" to serial monitor
        lcd.setCursor(14, 1);             // Set cursor to second row, fifteenth column
        lcd.print(counter);               // Display total count on LCD

        if (counter > 9 || counter < 0)
        {                // Check if counter is out of bounds
            delay(100);  // Delay for stability
            lcd.clear(); // Clear LCD display
        }
        delay(2000); // Delay for 2 seconds
        lcd.clear(); // Clear LCD display
    }
    else
    {
        digitalWrite(led, HIGH);     // Turn off LED
        lcd.clear();                 // Clear LCD display
        analogWrite(BUZZER_PIN, 0);  // Turn off the buzzer
        lcd.print("IN: ");           // Display "IN:" on LCD
        lcd.setCursor(8, 0);         // Set cursor to first row, ninth column
        lcd.print("Out: ");          // Display "Out:" on LCD
        lcd.setCursor(0, 1);         // Set cursor to second row, first column
        lcd.print("Total Inside: "); // Display "Total Inside:" on LCD

        measureDistanceAndUpdateLCD(); // Call function to measure distance and update LCD
        graph_ready(gas,FlameValue,vib,temp,inside,outside,counter);
    }
    delay(500); // Delay for stability
}