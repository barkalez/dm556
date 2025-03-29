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

// Create an AccelStepper instance
// The first parameter is the interface type (1 = driver step/dir interface)
AccelStepper stepper(1, PULSE_PIN, DIR_PIN);

// Variables para movimiento continuo
bool continuousMovement = false;
int continuousDirection = 1;  // 1 para avanzar, -1 para retroceder
float continuousSpeed = 3200;

// Variables para control de parada suave
long stopPosition = 0;
bool performingStopSequence = false;
bool returningToStopPosition = false;

// Añade estas variables para el movimiento acelerado
long targetPosition = 0;
bool useAcceleratedMovement = false;

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
    // Procesar datos Bluetooth
    while (Serial.available()) {
      char received = Serial.read();
      receivedData += received;
      if (received == '\n') {
        parseGCode(receivedData);
        receivedData = "";
      }
    }
  }

  // Manejo del movimiento continuo
  if (continuousMovement) {
    // Verificar límites
    if ((continuousDirection > 0 && stepper.currentPosition() < targetPosition) || 
        (continuousDirection < 0 && stepper.currentPosition() > targetPosition)) {
      
      // Cambiamos esta parte: en lugar de runSpeed(), usamos run() para respetar la aceleración
      if (useAcceleratedMovement) {
        stepper.run(); // Este método respeta la aceleración
        
        // Si estamos cerca del objetivo, configurar un nuevo objetivo más lejano
        if (abs(stepper.distanceToGo()) < 10000) {
          if (continuousDirection > 0) {
            targetPosition = stepper.currentPosition() + 1000000;
          } else {
            targetPosition = stepper.currentPosition() - 1000000;
          }
          stepper.moveTo(targetPosition);
        }
      } else {
        // Modo de velocidad constante (antiguo)
        stepper.runSpeed();
      }
    } else {
      // Detiene el movimiento si alcanza límites
      continuousMovement = false;
      useAcceleratedMovement = false;
      stepper.stop();
      Serial.println("Límite alcanzado, movimiento detenido");
      Serial.print("POS:");
      Serial.println(stepper.currentPosition());
    }
  } 
  // Manejo de la secuencia de parada suave
  else if (performingStopSequence) {
    // Si el motor ya se detuvo después de la desaceleración
    if (stepper.distanceToGo() == 0 && stepper.speed() == 0) {
      performingStopSequence = false;
      returningToStopPosition = true;
      
      // Configuramos parámetros para un movimiento de retorno preciso y suave
      stepper.setMaxSpeed(1000); // Velocidad más lenta para mayor precisión
      stepper.setAcceleration(500); // Aceleración suave
      stepper.moveTo(stopPosition); // Movemos a la posición exacta de parada
      
      Serial.println("Desaceleración completa, retornando a posición exacta");
    }
    else {
      // Continuamos con la desaceleración
      stepper.run();
    }
  }
  // Manejo del retorno a la posición exacta
  else if (returningToStopPosition) {
    // Si llegamos a la posición deseada
    if (stepper.distanceToGo() == 0) {
      returningToStopPosition = false;
      Serial.println("Retorno a posición exacta completado");
      Serial.print("POS:");
      Serial.println(stepper.currentPosition());
    }
    else {
      // Continuamos con el movimiento de retorno
      stepper.run();
    }
  }
  else {
    // Comportamiento normal para movimientos precisos
    stepper.run();
  }
}

void parseGCode(String gcode) {
  gcode.trim();
  Serial.print("Procesando G-code: ");
  Serial.println(gcode);

  // Comando para movimiento continuo hacia adelante
  if (gcode.startsWith("CONT+")) {
    continuousMovement = true;
    continuousDirection = 1;
    performingStopSequence = false;
    returningToStopPosition = false;
    
    // Procesar velocidad (F)
    int indexF = gcode.indexOf('F');
    if (indexF != -1) {
      int indexA = gcode.indexOf('A');
      if (indexA != -1) {
        continuousSpeed = gcode.substring(indexF + 1, indexA).toFloat();
      } else {
        continuousSpeed = gcode.substring(indexF + 1).toFloat();
      }
    } else {
      continuousSpeed = 3200;
    }
    
    // Procesar aceleración (A)
    int indexA = gcode.indexOf('A');
    long acceleration = 2000; // Valor por defecto
    if (indexA != -1) {
      acceleration = gcode.substring(indexA + 1).toFloat();
    }
    
    // Configuramos el movimiento acelerado
    stepper.setAcceleration(acceleration);
    stepper.setMaxSpeed(continuousSpeed);
    
    // Modo acelerado: establecer un objetivo lejano en la dirección correcta
    if (continuousDirection > 0) {
      targetPosition = stepper.currentPosition() + 1000000; // Un número grande hacia adelante
    } else {
      targetPosition = stepper.currentPosition() - 1000000; // Un número grande hacia atrás
    }
    stepper.moveTo(targetPosition);
    useAcceleratedMovement = true;
    
    Serial.print("Iniciando movimiento continuo acelerado con velocidad máxima ");
    Serial.print(continuousSpeed);
    Serial.print(" y aceleración ");
    Serial.println(acceleration);
    return;
  }

  // Comando para movimiento continuo hacia atrás (similar al anterior)
  if (gcode.startsWith("CONT-")) {
    continuousMovement = true;
    continuousDirection = -1;
    performingStopSequence = false;
    returningToStopPosition = false;
    
    // Procesar velocidad (F)
    int indexF = gcode.indexOf('F');
    if (indexF != -1) {
      int indexA = gcode.indexOf('A');
      if (indexA != -1) {
        continuousSpeed = gcode.substring(indexF + 1, indexA).toFloat();
      } else {
        continuousSpeed = gcode.substring(indexF + 1).toFloat();
      }
    } else {
      continuousSpeed = 3200;
    }
    
    // Procesar aceleración (A)
    int indexA = gcode.indexOf('A');
    long acceleration = 2000; // Valor por defecto
    if (indexA != -1) {
      acceleration = gcode.substring(indexA + 1).toFloat();
    }
    
    // Configuramos el movimiento acelerado
    stepper.setAcceleration(acceleration);
    stepper.setMaxSpeed(continuousSpeed);
    
    // Modo acelerado: establecer un objetivo lejano en la dirección correcta
    targetPosition = stepper.currentPosition() - 1000000; // Un número grande hacia atrás
    stepper.moveTo(targetPosition);
    useAcceleratedMovement = true;
    
    Serial.print("Iniciando movimiento continuo acelerado con velocidad máxima ");
    Serial.print(continuousSpeed);
    Serial.print(" y aceleración ");
    Serial.println(acceleration);
    return;
  }

  // Comando para detener movimiento continuo INMEDIATAMENTE
  if (gcode.startsWith("STOP")) {
    if (continuousMovement) {
      // Detenemos el modo de movimiento continuo
      continuousMovement = false;
      
      // Deshabilitamos cualquier secuencia de parada suave
      performingStopSequence = false;
      returningToStopPosition = false;
      useAcceleratedMovement = false;
      
      // Paramos inmediatamente el motor
      stepper.setSpeed(0); // Detiene la velocidad
      stepper.stop(); // Detiene cualquier movimiento planificado
      stepper.disableOutputs(); // Desactiva las bobinas para parada inmediata
      delay(5); // Pequeña pausa
      stepper.enableOutputs(); // Reactiva el motor
      
      // Actualizamos la posición actual para mantener la precisión
      long currentPos = stepper.currentPosition();
      stepper.setCurrentPosition(currentPos);
      
      // Notificar la posición actual
      Serial.println("Movimiento detenido en seco");
      Serial.print("POS:");
      Serial.println(stepper.currentPosition());
    }
    return;
  }

  // Comando G1 para movimiento a posición específica
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

    // Procesar aceleración (A)
    int indexA = gcode.indexOf('A');
    if (indexA != -1) {
      long acceleration = gcode.substring(indexA + 1).toFloat();
      stepper.setAcceleration(acceleration);
      Serial.print("Aceleración configurada a: ");
      Serial.println(acceleration);
    }
  }

  if (gcode.startsWith("G28")) {
    // Comando de homing
    Serial.println("Iniciando homing...");
    
    // Primero mover hacia atrás hasta encontrar el home
    stepper.setSpeed(-1600); // Velocidad de homing
    while (digitalRead(ENDSTOP_PIN) == HIGH) {
      stepper.runSpeed();
    }
    stepper.stop();
    
    // Pequeña pausa para estabilizar
    delay(100);
    
    // Ahora mover hacia adelante hasta perder el home
    stepper.setSpeed(800); // Velocidad más lenta para precisión
    while (digitalRead(ENDSTOP_PIN) == LOW) {
      stepper.runSpeed();
    }
    stepper.stop();
    
    // Pequeña pausa para estabilizar
    delay(100);
    
    // Finalmente, mover hacia atrás hasta encontrar el home una vez más
    stepper.setSpeed(-400); // Velocidad muy lenta para máxima precisión
    while (digitalRead(ENDSTOP_PIN) == HIGH) {
      stepper.runSpeed();
    }
    stepper.stop();
    
    // Establecer la posición actual como 0
    stepper.setCurrentPosition(0);
    
    // Enviar comando especial para indicar que el homing está completo
    Serial.println("HOMING_COMPLETE");
    
    return;
  }
}