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
 *   {"position": 12345, "speed": 123.45, "acceleration": 12.34, "timestamp": 1234567890}
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

// Velocity filtering parameters
#define BUFFER_SIZE 10               // 10 samples at 200Hz = 50ms window
#define FILTER_ALPHA 0.3             // Exponential moving average coefficient

// Encoder state
volatile long encoderPosition = 0;
long lastPosition = 0;

// Velocity buffering (circular buffer for 50ms window)
long positionBuffer[BUFFER_SIZE];
unsigned long timeBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

// Timing for speed/acceleration calculation
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Speed and acceleration
float rawSpeed = 0.0;        // Instantaneous counts/second
float filteredSpeed = 0.0;   // Low-pass filtered counts/second
float lastFilteredSpeed = 0.0;
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
  
  // Initialize velocity buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    positionBuffer[i] = 0;
    timeBuffer[i] = 0;
  }
  
  // Wait for serial connection
  delay(100);
}


void loop() {
  currentTime = millis();
  
  // Publish at fixed rate (200 Hz)
  if (currentTime - lastTime >= PUBLISH_INTERVAL_MS) {
    // Add current position and time to circular buffer
    positionBuffer[bufferIndex] = encoderPosition;
    timeBuffer[bufferIndex] = micros();
    
    // Increment buffer index (circular)
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) {
      bufferCount++;
    }
    
    // Only calculate velocity once we have enough buffer data (50ms worth)
    if (bufferCount >= BUFFER_SIZE) {
      // Calculate velocity over entire buffer window (50ms)
      int oldestIndex = bufferIndex;  // Oldest sample in circular buffer
      int newestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      
      long positionDelta = positionBuffer[newestIndex] - positionBuffer[oldestIndex];
      unsigned long timeDelta = timeBuffer[newestIndex] - timeBuffer[oldestIndex];
      
      // Calculate raw speed (counts/second) over 50ms window
      if (timeDelta > 0) {
        rawSpeed = (positionDelta * 1000000.0) / timeDelta;  // Convert microseconds to seconds
      } else {
        rawSpeed = 0.0;
      }
      
      // Apply exponential moving average filter for smoothing
      // filteredSpeed = alpha * rawSpeed + (1 - alpha) * lastFilteredSpeed
      filteredSpeed = FILTER_ALPHA * rawSpeed + (1.0 - FILTER_ALPHA) * lastFilteredSpeed;
      
      // Calculate acceleration from filtered speed
      float dt = (currentTime - lastTime) / 1000.0;
      if (dt > 0) {
        acceleration = (filteredSpeed - lastFilteredSpeed) / dt;
      } else {
        acceleration = 0.0;
      }
      
      // Update state
      lastFilteredSpeed = filteredSpeed;
    } else {
      // Not enough data yet, output zero
      filteredSpeed = 0.0;
      acceleration = 0.0;
    }
    
    lastTime = currentTime;
    
    // Publish JSON message with timestamp in microseconds
    // Format: {"position": 12345, "speed": 123.45, "acceleration": 12.34, "timestamp": 1234567890}
    Serial.print("{\"position\":");
    Serial.print(encoderPosition);
    Serial.print(",\"speed\":");
    Serial.print(filteredSpeed, 2);  // 2 decimal places - filtered velocity
    Serial.print(",\"acceleration\":");
    Serial.print(acceleration, 2);
    Serial.print(",\"timestamp\":");
    Serial.print(micros());  // Timestamp in microseconds
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
