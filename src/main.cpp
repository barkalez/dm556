#include <Arduino.h>
#include <AccelStepper.h>

// Pines para DM556
#define PULSE_PIN 2   // D2 como PUL+
#define DIR_PIN 3     // D3 como DIR+
#define ENABLE_PIN 4  // D4 como ENA+

// Pines para HC-05
#define HC05_EN 5     // D5 como EN del HC-05
#define HC05_STATE 6  // D6 como STATE del HC-05

// Pin para endstop
#define ENDSTOP_PIN 7 // D7 como endstop

// Configuración de AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, PULSE_PIN, DIR_PIN);

// Límite máximo de pasos
const long MAX_STEPS = 40000;

void parseGCode(String gcode);

String receivedData = "";

void setup() {
  Serial.begin(115200); // Baud rate configurado en el HC-05
  while (!Serial);
  Serial.println("Arduino Nano con HC-05 iniciado");

  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(HC05_EN, OUTPUT);
  digitalWrite(HC05_EN, LOW);
  pinMode(HC05_STATE, INPUT);

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(3200);
  stepper.setAcceleration(1600);
  stepper.setCurrentPosition(0);
}

void loop() {
  if (digitalRead(HC05_STATE) == HIGH) {
    while (Serial.available()) {
      char received = Serial.read();
      receivedData += received;
      if (received == '\n') {
        parseGCode(receivedData);
        receivedData = "";
      }
    }
  }
  stepper.run();
}

void parseGCode(String gcode) {
  gcode.trim();
  Serial.print("Procesando G-code: ");
  Serial.println(gcode);

  if (gcode.startsWith("G1")) {
    int indexX = gcode.indexOf('X');
    int indexF = gcode.indexOf('F');
    float x = stepper.currentPosition(); // Posición absoluta por defecto
    float f = stepper.maxSpeed();

    if (indexX != -1) {
      x = gcode.substring(indexX + 1).toFloat();
    }
    if (indexF != -1) {
      f = gcode.substring(indexF + 1).toFloat();
    }

    long targetPosition = x; // Usar X como posición absoluta

    if (targetPosition > MAX_STEPS) {
      Serial.println("Límite máximo alcanzado (40000 pasos)");
      Serial.println("END");
      stepper.stop();
      return;
    }
    if (targetPosition < 0) {
      Serial.println("Límite mínimo alcanzado (0 pasos)");
      stepper.stop();
      return;
    }

    Serial.print("Mover a posición absoluta X: ");
    Serial.println(x);
    Serial.print("Velocidad: ");
    Serial.println(f);
    Serial.print("Posición inicial: ");
    Serial.println(stepper.currentPosition());

    stepper.setSpeed(f);
    stepper.moveTo(targetPosition); // Mover a posición absoluta
    stepper.runToPosition();

    Serial.println("Movimiento completado");
    Serial.print("Posición final: ");
    Serial.println(stepper.currentPosition());
    Serial.print("POS:");
    Serial.println(stepper.currentPosition());
  }

  if (gcode.startsWith("G28")) {
    Serial.println("Moviendo a la posición de origen (homing)");

    stepper.setSpeed(3200);
    stepper.move(-100000);

    while (digitalRead(ENDSTOP_PIN) == HIGH && stepper.distanceToGo() != 0) {
      stepper.run();
    }

    stepper.stop();
    Serial.println("Endstop detectado");

    stepper.setSpeed(3200);
    stepper.move(200);
    stepper.runToPosition();
    Serial.println("Retrocedido 200 pasos");

    stepper.setCurrentPosition(0);
    Serial.println("Posición absoluta establecida a 0");
    Serial.println("pos0");
    Serial.print("POS:");
    Serial.println(stepper.currentPosition());
  }
}