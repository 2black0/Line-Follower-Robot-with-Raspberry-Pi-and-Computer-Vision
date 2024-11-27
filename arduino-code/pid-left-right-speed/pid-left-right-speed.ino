// Motor Left (Motor A) Pins
int MotorLeftPin1 = 13; // IN1
int MotorLeftPin2 = 12; // IN2
int EnableLeftPin = 2;  // PWM for Motor Left

// Motor Right (Motor B) Pins
int MotorRightPin1 = 27; // IN3
int MotorRightPin2 = 26; // IN4
int EnableRightPin = 4;  // PWM for Motor Right

// Encoder Pins
int EncoderLeftPinA = 33; // Left Encoder Phase A
int EncoderLeftPinB = 32; // Left Encoder Phase B
int EncoderRightPinA = 18; // Right Encoder Phase A
int EncoderRightPinB = 5;  // Right Encoder Phase B

// Encoder Variables
volatile int LastEncodedLeft = 0;     // Previous encoder state (Left)
volatile long EncoderValueLeft = 0;   // Current encoder count (Left)
volatile int LastEncodedRight = 0;    // Previous encoder state (Right)
volatile long EncoderValueRight = 0;  // Current encoder count (Right)
volatile long PrevEncoderValueLeft = 0;
volatile long PrevEncoderValueRight = 0;

const float EncoderResolution = 11.0; // Encoder resolution: 11 pulses per rotation
float RpsLeft = 0.0;  // Rotations per second for Left motor
float RpsRight = 0.0; // Rotations per second for Right motor

// PID Parameters for Left Motor
float KpLeft = 1.5; 
float KiLeft = 1.8; 
float KdLeft = 0.01; 

float IntegralLeft = 0.0;  
float PrevErrorLeft = 0.0; 
int PwmLeft = 0;  

// PID Parameters for Right Motor
float KpRight = 1.45; 
float KiRight = 1.75; 
float KdRight = 0.01; 

float IntegralRight = 0.0;  
float PrevErrorRight = 0.0; 
int PwmRight = 0;  

const float RpsInterval = 100; // Interval for RPS calculation in milliseconds
unsigned long LastRpsCalcTime = 0;

// Target RPS for motors
float SetpointRpsLeft = 0.0;  // Desired RPS for Left motor
float SetpointRpsRight = 0.0; // Desired RPS for Right motor

void setup() {
  InitializePins();
  AttachEncoderInterrupts();
  Serial.begin(115200);
  Serial.println("Setup complete.");
}

void loop() {
  unsigned long currentTime = millis();

  // Read and parse data from Raspberry Pi
  if (Serial.available() > 0) {
    ParseSerialData();
  }

  // Calculate RPS every RpsInterval milliseconds
  if (currentTime - LastRpsCalcTime >= RpsInterval) {
    LastRpsCalcTime = currentTime;

    // Calculate RPS for both motors
    CalculateRps();

    // Calculate individual PID for Left and Right motors
    PwmLeft = CalculatePid(SetpointRpsLeft, RpsLeft, KpLeft, KiLeft, KdLeft, IntegralLeft, PrevErrorLeft);
    PwmRight = CalculatePid(SetpointRpsRight, RpsRight, KpRight, KiRight, KdRight, IntegralRight, PrevErrorRight);

    // Debugging output
    DebugMotorInfo();  // For Serial Monitor
    //PlotMotorData();   // For Arduino Plotter

    // Drive the motors with updated PWM values
    DriveMotor(MotorLeftPin1, MotorLeftPin2, EnableLeftPin, PwmLeft, true);
    DriveMotor(MotorRightPin1, MotorRightPin2, EnableRightPin, PwmRight, true);
  }
}

// Parse serial data and update motor setpoints
void ParseSerialData() {
  static String inputString = ""; // Buffer for incoming serial data
  char incomingChar;

  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    if (incomingChar == '\n') {
      // Parse the received message
      if (inputString.startsWith("L") && inputString.indexOf("R") > 0) {
        int leftIndex = inputString.indexOf("L") + 1;
        int rightIndex = inputString.indexOf("R") + 1;

        // Ensure valid indices for parsing
        if (rightIndex > leftIndex && rightIndex < inputString.length()) {
          // Extract left and right speeds from the message
          String leftSpeedStr = inputString.substring(leftIndex, inputString.indexOf("R"));
          String rightSpeedStr = inputString.substring(rightIndex);

          // Trim the extracted strings
          leftSpeedStr.trim();
          rightSpeedStr.trim();

          // Convert to float and update setpoints
          SetpointRpsLeft = leftSpeedStr.toFloat();
          SetpointRpsRight = rightSpeedStr.toFloat();

          Serial.print("Setpoint Updated - Left: ");
          Serial.print(SetpointRpsLeft);
          Serial.print(" RPS, Right: ");
          Serial.println(SetpointRpsRight);
        } else {
          Serial.println("Error: Invalid data format.");
        }
      } else {
        Serial.println("Error: Missing 'L' or 'R' in input.");
      }
      inputString = ""; // Clear the buffer
    } else {
      inputString += incomingChar; // Append to buffer
    }
  }
}

// Initialize pins for motors and encoders
void InitializePins() {
  // Motor Pins
  pinMode(MotorLeftPin1, OUTPUT);
  pinMode(MotorLeftPin2, OUTPUT);
  pinMode(EnableLeftPin, OUTPUT);
  pinMode(MotorRightPin1, OUTPUT);
  pinMode(MotorRightPin2, OUTPUT);
  pinMode(EnableRightPin, OUTPUT);

  // Encoder Pins
  pinMode(EncoderLeftPinA, INPUT_PULLUP);
  pinMode(EncoderLeftPinB, INPUT_PULLUP);
  pinMode(EncoderRightPinA, INPUT_PULLUP);
  pinMode(EncoderRightPinB, INPUT_PULLUP);
}

// Attach encoder interrupts
void AttachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(EncoderLeftPinA), UpdateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderLeftPinB), UpdateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderRightPinA), UpdateEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderRightPinB), UpdateEncoderRight, CHANGE);
}

// Calculate Rotations Per Second (RPS)
void CalculateRps() {
  long deltaLeft = EncoderValueLeft - PrevEncoderValueLeft;
  long deltaRight = EncoderValueRight - PrevEncoderValueRight;

  RpsLeft = (deltaLeft / EncoderResolution) / (RpsInterval / 1000.0);
  RpsRight = (deltaRight / EncoderResolution) / (RpsInterval / 1000.0);

  PrevEncoderValueLeft = EncoderValueLeft;
  PrevEncoderValueRight = EncoderValueRight;
}

// Calculate PID output for a motor
int CalculatePid(float setpoint, float measured, float Kp, float Ki, float Kd, float &integral, float &prevError) {
  float error = setpoint - measured;
  integral += error * (RpsInterval / 1000.0);
  float derivative = (error - prevError) / (RpsInterval / 1000.0);
  prevError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return constrain((int)output, 0, 255);
}

// Drive motor with PWM
void DriveMotor(int dirPin1, int dirPin2, int enablePin, int pwmValue, bool forward) {
  if (forward) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  }
  analogWrite(enablePin, pwmValue);
}

// Debug motor information
void DebugMotorInfo() {
  Serial.print("Setpoint Left: ");
  Serial.print(SetpointRpsLeft);
  Serial.print(" | RPS Left: ");
  Serial.print(RpsLeft);
  Serial.print(" | PWM Left: ");
  Serial.print(PwmLeft);
  Serial.print(" || Setpoint Right: ");
  Serial.print(SetpointRpsRight);
  Serial.print(" | RPS Right: ");
  Serial.print(RpsRight);
  Serial.print(" | PWM Right: ");
  Serial.println(PwmRight);
}

// Plot motor data for Arduino Plotter
void PlotMotorData() {
  // Comment this block to disable Arduino Plotter debugging
  //Serial.print("Setpoint Left: ");
  Serial.print(SetpointRpsLeft);
  Serial.print(" ");
  //Serial.print("RPS Left:");
  Serial.print(RpsLeft);
  Serial.print(" ");
  //Serial.print("Setpoint Right: ");
  Serial.print(SetpointRpsRight);
  Serial.print(" ");
  //Serial.print("RPS Right: ");
  Serial.println(RpsRight);
}

// Update encoder for Left Motor
void UpdateEncoderLeft() {
  int msb = digitalRead(EncoderLeftPinA);
  int lsb = digitalRead(EncoderLeftPinB);
  int encoded = (msb << 1) | lsb;
  int sum = (LastEncodedLeft << 2) | encoded;

  noInterrupts();
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    EncoderValueLeft = EncoderValueLeft - 1;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    EncoderValueLeft = EncoderValueLeft + 1;
  }
  interrupts();

  LastEncodedLeft = encoded;
}

// Update encoder for Right Motor
void UpdateEncoderRight() {
  int msb = digitalRead(EncoderRightPinA);
  int lsb = digitalRead(EncoderRightPinB);
  int encoded = (msb << 1) | lsb;
  int sum = (LastEncodedRight << 2) | encoded;

  noInterrupts();
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    EncoderValueRight = EncoderValueRight + 1;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    EncoderValueRight = EncoderValueRight - 1;
  }
  interrupts();

  LastEncodedRight = encoded;
}