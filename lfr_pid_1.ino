#include "BluetoothSerial.h" 

BluetoothSerial SerialBT;  // Initialize Bluetooth

// Motor Driver Pins
#define STBY 13  
#define AIN1 27
#define AIN2 26
#define PWMA 14
#define BIN1 25
#define BIN2 33
#define PWMB 32

// IR Sensor Pins
#define NUM_SENSORS 8
int ir_pins[NUM_SENSORS] = {34, 35, 36, 39, 4, 16, 17, 18};

// PID Constants (modifiable via Bluetooth)
float Kp = 20.0, Ki = 0.0, Kd = 8.0;
float lastError = 0, integral = 0;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_PID_Tuner");  // Bluetooth device name

    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    digitalWrite(STBY, HIGH);

    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(ir_pins[i], INPUT);
    }

    Serial.println("Bluetooth Ready! Send 'P=xx', 'I=xx', or 'D=xx' to adjust PID.");
}

void loop() {
    // Read sensor values
    int sensor_values[NUM_SENSORS];
    int position = 0, sum = 0, active_sensors = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = digitalRead(ir_pins[i]);
        if (sensor_values[i] == 1) {
            sum += i;
            active_sensors++;
        }
    }
    position = (active_sensors > 0) ? (sum / active_sensors) : -1;

    // PID Control
    float error = position - (NUM_SENSORS / 2);
    integral += error;
    float derivative = error - lastError;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Adjust motor speeds
    int leftSpeed = constrain(120 - correction, 0, 255);
    int rightSpeed = constrain(120 + correction, 0, 255);
    moveMotors(leftSpeed, rightSpeed);

    // Check for Bluetooth messages
    if (SerialBT.available()) {
        String msg = SerialBT.readString();
        msg.trim();

        if (msg.startsWith("P=")) {
            Kp = msg.substring(2).toFloat();
            Serial.println("Updated Kp: " + String(Kp));
        } else if (msg.startsWith("I=")) {
            Ki = msg.substring(2).toFloat();
            Serial.println("Updated Ki: " + String(Ki));
        } else if (msg.startsWith("D=")) {
            Kd = msg.substring(2).toFloat();
            Serial.println("Updated Kd: " + String(Kd));
        }
    }

    delay(50);
}

void moveMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed > rightSpeed) {
        turnLeft(leftSpeed, rightSpeed);
    } else if (rightSpeed > leftSpeed) {
        turnRight(leftSpeed, rightSpeed);
    } else {
        moveForward(leftSpeed);
    }
}

void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void turnLeft(int leftSpeed, int rightSpeed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, leftSpeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
}

void turnRight(int leftSpeed, int rightSpeed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, rightSpeed);
}
