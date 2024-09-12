// Pines del driver L298N
const int in1 = 4;
const int in2 = 5;
const int enA = 6; // Pin PWM

// Pines del encoder
const int encoderPinA = 3;
const int encoderPinB = 2;

// Variables del encoder
long encoderPos = 0;
long lastEncoderPos = 0;
unsigned long lastTime = 0;
bool lastStateA;

// PID variables
float velocidad = 0; // Velocidad medida en pulsos/segundo
float velocidadReferencia = 400; // Velocidad deseada (fija en pulsos/segundo)

float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float controlSignal = 0;

// Parámetros PID
float KP = 0.1;  // Ganancia proporcional
float KI = 0.1;  // Ganancia integral
float KD = 0.0001; // Ganancia derivativa

// Limites de PWM
int pwmValue = 0;
int pwmMin = 0;
int pwmMax = 255;

void setup() {
  // Configura los pines del driver L298N
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  // Configura los pines del encoder como entradas
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Inicializa la variable de estado previo del encoder
  lastStateA = digitalRead(encoderPinA);

  // Inicializa comunicación serial
  Serial.begin(9600);
  Serial.println("VelRef,VelAct,PWM,Error");
}

void loop() {
  // Lee el estado actual del pin A del encoder
  bool currentStateA = digitalRead(encoderPinA);

  // Si hay un cambio de estado en el pin A del encoder
  if (currentStateA != lastStateA) {
    // Verifica el estado del pin B para determinar la dirección
    if (digitalRead(encoderPinB) != currentStateA) {
      encoderPos++;  // Rotación hacia adelante
    } else {
      encoderPos--;  // Rotación hacia atrás
    }

    // Actualiza el estado anterior del pin A
    lastStateA = currentStateA;
  }

  // Calcular la velocidad cada 100 ms
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastTime;

  if (timeDiff > 100) {
    // Calcula la diferencia de pulsos
    long encoderDiff = encoderPos - lastEncoderPos;

    // Calcula la velocidad (pulsos por segundo)
    velocidad = (float)encoderDiff / timeDiff * 1000.0; // velocidad en pulsos/segundo

    // Control PID
    error = velocidadReferencia - velocidad; // Error entre referencia y medida

    integral += error * timeDiff / 1000.0; // Integración del error
    derivative = (error - lastError) / (timeDiff / 1000.0); // Derivada del error

    // Señal de control
    controlSignal = KP * error + KI * integral + KD * derivative;

    // Actualiza el último error
    lastError = error;
    lastEncoderPos = encoderPos;
    lastTime = currentTime;

    // Limita la señal de control para estar entre los valores permitidos de PWM
    pwmValue = constrain(controlSignal, pwmMin, pwmMax);

    // Aplica la señal de control al motor
    if (pwmValue > 0) {
      digitalWrite(in1, HIGH);  // Avance
      digitalWrite(in2, LOW);
    } else {
      digitalWrite(in1, LOW);   // Retroceso
      digitalWrite(in2, HIGH);
    }

    analogWrite(enA, abs(pwmValue));

    // Imprime la velocidad y otros datos para monitoreo

    Serial.print(velocidadReferencia);
    Serial.print(",");
    Serial.print(velocidad);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(error);
    Serial.println(",");
  }
}
