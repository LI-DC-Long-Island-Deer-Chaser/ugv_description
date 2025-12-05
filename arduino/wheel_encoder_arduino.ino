/*
 * Wheel Encoder Arduino Code
 * 
 * Reads A1230K quadrature encoder with magnetic ring (12 alternating poles)
 * Publishes JSON serial data at 200Hz to ROS 2 node
 * 
 * Hardware:
 *   - A1230K Hall effect encoder (36 counts/revolution)
 *   - Magnetic encoder ring (12 poles)
 *   - Connected to driveshaft (3.09:1 gear ratio to wheels)
 * 
 * Output Format (JSON):
 *   {"position": 12345, "speed": 123.45, "acceleration": 12.34}
 * 
 * Wiring:
 *   - Encoder A (Phase A) -> Pin 2 (interrupt 0)
 *   - Encoder B (Phase B) -> Pin 3 (interrupt 1)
 *   - Encoder VCC -> 5V
 *   - Encoder GND -> GND
 */

// Encoder pins (must be interrupt-capable on most Arduinos)
#define ENCODER_A_PIN 2  // Interrupt 0
#define ENCODER_B_PIN 3  // Interrupt 1

// Timing
#define PUBLISH_RATE_HZ 200
#define PUBLISH_INTERVAL_MS (1000 / PUBLISH_RATE_HZ)

// Encoder state
volatile long encoderPosition = 0;
long lastPosition = 0;
long lastSpeed = 0;

// Timing for speed/acceleration calculation
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Speed and acceleration
float speed = 0.0;           // counts/second
float acceleration = 0.0;    // counts/second^2


void setup() {
  // Initialize serial at 115200 baud (matches ROS 2 node)
  Serial.begin(115200);
  
  // Configure encoder pins as inputs with pull-up resistors
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  
  // Attach interrupts for quadrature decoding
  // CHANGE mode triggers on any logic change (rising or falling edge)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR_B, CHANGE);
  
  // Initialize timing
  lastTime = millis();
  
  // Wait for serial connection
  delay(100);
}


void loop() {
  currentTime = millis();
  
  // Publish at fixed rate (200 Hz)
  if (currentTime - lastTime >= PUBLISH_INTERVAL_MS) {
    // Calculate time delta (seconds)
    float dt = (currentTime - lastTime) / 1000.0;
    
    // Calculate speed (counts/second)
    long positionDelta = encoderPosition - lastPosition;
    float newSpeed = positionDelta / dt;
    
    // Calculate acceleration (counts/second^2)
    acceleration = (newSpeed - speed) / dt;
    
    // Update state
    speed = newSpeed;
    lastPosition = encoderPosition;
    lastTime = currentTime;
    
    // Publish JSON message
    // Format: {"position": 12345, "speed": 123.45, "acceleration": 12.34}
    Serial.print("{\"position\":");
    Serial.print(encoderPosition);
    Serial.print(",\"speed\":");
    Serial.print(speed, 2);  // 2 decimal places
    Serial.print(",\"acceleration\":");
    Serial.print(acceleration, 2);
    Serial.println("}");
  }
}


/*
 * Interrupt Service Routine for Encoder Phase A
 * Uses quadrature decoding to determine direction
 */
void encoderISR_A() {
  // Read both encoder phases
  bool A = digitalRead(ENCODER_A_PIN);
  bool B = digitalRead(ENCODER_B_PIN);
  
  // Quadrature decoding logic
  // If A changed and A == B, we're moving backward
  // If A changed and A != B, we're moving forward
  if (A == B) {
    encoderPosition--;  // Backward
  } else {
    encoderPosition++;  // Forward
  }
}


/*
 * Interrupt Service Routine for Encoder Phase B
 * Uses quadrature decoding to determine direction
 */
void encoderISR_B() {
  // Read both encoder phases
  bool A = digitalRead(ENCODER_A_PIN);
  bool B = digitalRead(ENCODER_B_PIN);
  
  // Quadrature decoding logic
  // If B changed and A != B, we're moving backward
  // If B changed and A == B, we're moving forward
  if (A != B) {
    encoderPosition--;  // Backward
  } else {
    encoderPosition++;  // Forward
  }
}
