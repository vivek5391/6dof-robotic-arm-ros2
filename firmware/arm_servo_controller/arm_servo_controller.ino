#if defined(ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

const int NUM_SERVOS = 6;

#if defined(ESP32)
  const int SERVO_PINS[NUM_SERVOS] = {13, 12, 14, 27, 26, 25};
#else
  const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};
#endif

const int MIN_DEG[NUM_SERVOS] = {  0,  60,  25,   0,  30,  61};
const int MIN_DEG[NUM_SERVOS] = {  0,  60,  25,   0,  30,  90};
const int MAX_DEG[NUM_SERVOS] = {180, 165, 160, 180, 150, 127};

Servo servos[NUM_SERVOS];
int targetAngles[NUM_SERVOS]    = {90, 90, 90, 90, 90, 90};
float currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

String inputBuffer = "";
unsigned long lastStepTime = 0;

void setup() {
  Serial.begin(115200);
  inputBuffer.reserve(64);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(90);
  }
}

void parseCommand(String line) {
  int idx = 0;
  int startPos = 0;
  for (int i = 0; i <= line.length() && idx < NUM_SERVOS; i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      String token = line.substring(startPos, i);
      token.trim();
      if (token.length() > 0) {
        int val = constrain(token.toInt(), MIN_DEG[idx], MAX_DEG[idx]);
        targetAngles[idx] = val;
        idx++;
      }
      startPos = i + 1;
    }
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  if (millis() - lastStepTime >= 10) {
    lastStepTime = millis();
    for (int i = 0; i < NUM_SERVOS; i++) {
      float diff = targetAngles[i] - currentAngles[i];
      if (abs(diff) > 0.2f) {
        currentAngles[i] += diff * 0.25f;
        servos[i].write((int)round(currentAngles[i]));
      }
    }
  }
}
