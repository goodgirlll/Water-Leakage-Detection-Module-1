// Define pins for flow sensors
#define FLOW_SENSOR_1 2
#define FLOW_SENSOR_2 3

// Define pins for LED, buzzer and solenoid valve
#define LED_PIN 4
#define BUZZER_PIN 5
#define VALVE_PIN 6


// Define threshold for leak detection
#define LEAK_THRESHOLD 0.1

// Define variables for flow rate calculation
volatile int pulseCount1 = 0; // count pulses from flow sensor 1
volatile int pulseCount2 = 0; // count pulses from flow sensor 2
float flowRate1 = 0.0; // flow rate from sensor 1 in L/min
float flowRate2 = 0.0; // flow rate from sensor 2 in L/min
float calibrationFactor = 4.5; // calibration factor for flow sensor
unsigned long currentTime;
unsigned long previousTime;
unsigned long interval = 1000; // interval in milliseconds

// Define variable for leak detection
bool leakDetected = false;


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pins as inputs or outputs
  pinMode(FLOW_SENSOR_1, INPUT);
  pinMode(FLOW_SENSOR_2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  // Initialize interrupts for flow sensors
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_1), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_2), pulseCounter2, FALLING);
}

void loop() {
  // Get current time
  currentTime = millis();

  // Check if interval has passed
  if (currentTime - previousTime >= interval) {
    // Calculate flow rate for each sensor
    flowRate1 = ((1000.0 / (currentTime - previousTime)) * pulseCount1) / calibrationFactor;
    flowRate2 = ((1000.0 / (currentTime - previousTime)) * pulseCount2) / calibrationFactor;

    // Reset pulse counts and previous time
    pulseCount1 = 0;
    pulseCount2 = 0;
    previousTime = currentTime;

    // Print flow rates to serial monitor
    Serial.print("Flow rate 1: ");
    Serial.print(flowRate1, 2);
    Serial.println(" L/min");
    Serial.print("Flow rate 2: ");
    Serial.print(flowRate2, 2);
    Serial.println(" L/min");

    // Check if there is a significant difference between flow rates
    if (abs(flowRate1 - flowRate2) > LEAK_THRESHOLD) {
      // Set leak detected flag to true
      leakDetected = true;
      // Turn on LED and buzzer
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      // Close solenoid valve
      digitalWrite(VALVE_PIN, LOW);
    }
    else {
      // Set leak detected flag to false
      leakDetected = false;
      // Turn off LED and buzzer
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      // Open solenoid valve
      digitalWrite(VALVE_PIN, HIGH);
    }
  }
}

// Interrupt service routine for flow sensor 1
void pulseCounter1() {
  // Increment pulse count
  pulseCount1++;
}

// Interrupt service routine for flow sensor 2
void pulseCounter2() {
  // Increment pulse count
  pulseCount2++;
}
