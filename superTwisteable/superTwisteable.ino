// Pines
const int motorPWM = 33;  // Pin PWM para controlar la velocidad del motor (ENB)
const int motorIn1 = 25;  // Pin IN1 del L298N
const int motorIn2 = 26;  // Pin IN2 del L298N
const int encoderPinA = 32;  // Pin A del encoder
const int encoderPinB = 35;  // Pin B del encoder

// Variables del encoder
volatile int encoderCount = 0;
int lastEncoderCount = 0;
unsigned long lastTime = 0;
float currentSpeed = 0.0;

// Variables del control
float setpoint = 11.0;  // Velocidad deseada (en pulsos por segundo)
float u = 0.0;  // Señal de control
float integralSignE = 0.0;  // Integral del signo del error
float lastError = 0.0;

// Parámetros del controlador Super-Twisting
float k_ps = 10.0;  // Ganancia proporcional del Super-Twisting
float k_is = 1.0;  // Ganancia integral del Super-Twisting

void setup() {
  Serial.begin(115200);

  // Configuración de pines
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Configuración de interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // Calcular la velocidad del motor en pulsos por segundo
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime >= 100) {  // Actualización de velocidad cada 100 ms
    currentSpeed = (encoderCount - lastEncoderCount) * (1000.0 / elapsedTime);
    lastEncoderCount = encoderCount;
    lastTime = currentTime;

    // Calcular el error
    float error = setpoint - currentSpeed;

    // Primer término: k_ps * |e|^1/2 * sign(e)
    float term1 = k_ps * sqrt(abs(error)) * sign(error);

    // Segundo término: k_is * integral(sign(e) dt)
    integralSignE += sign(error) * (elapsedTime / 1000.0);  // Integral con respecto al tiempo
    float term2 = k_is * integralSignE;

    // Señal de control
    u = term1 + term2;

    // Saturar la señal de control a los valores de PWM (0-255)
    int pwmValue = constrain(abs(u), 0, 255);

    // Controlar la dirección del motor
    if (u > 0) {
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);  // Motor hacia adelante
    } else {
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, HIGH);  // Motor hacia atrás
    }

    // Aplicar la señal PWM al motor
    analogWrite(motorPWM, pwmValue);

    // Mostrar información en el monitor serial
    Serial.print("Velocidad actual: ");
    Serial.print(currentSpeed);
    Serial.print("  Setpoint: ");
    Serial.print(setpoint);
    Serial.print("  PWM: ");
    Serial.println(pwmValue);
  }
}

// Función de interrupción para el encoder
void updateEncoder() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Incrementar o decrementar el contador en función de la dirección del encoder
  if (stateA == stateB) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Función signo
int sign(float x) {
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}
