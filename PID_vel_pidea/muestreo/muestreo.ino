// Pines del driver L298N
const int in1 = 4;
const int in2 = 5;
const int enA = 6; // Pin PWM

// Pines del potenciómetro
const int potPin = A0;

// Pines del encoder
const int encoderPinA = 3;
const int encoderPinB = 2;

// Variables del encoder
long encoderPos = 0;
long lastEncoderPos = 0;
unsigned long lastTime = 0;
bool lastStateA;

// Velocidad
float velocidad = 0; // Velocidad en pulsos/segundo

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
  Serial.println("Setup completado.");
}

void loop() {

  unsigned long currentTime = millis();

  // Lee el valor del potenciómetro y mapea a un valor PWM (0-255)
  int potValue = analogRead(potPin);
  int pwmValue = map(potValue, 0, 1023, 0, 255);

    // Controla la velocidad del motor con PWM
  analogWrite(enA, abs(pwmValue));

  // Control de dirección del motor
 // if (pwmValue > 10) {
    digitalWrite(in1, HIGH); // Avance
    digitalWrite(in2, LOW);
  //} else if (pwmValue < -10) {
  //   digitalWrite(in1, LOW);  // Retroceso
  //   digitalWrite(in2, HIGH);
  // } else {
  //   digitalWrite(in1, LOW);  // Parada
  //   digitalWrite(in2, LOW);
  // }

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

  // Calcular la velocidad cada 10 ms
  unsigned long timeDiff = currentTime - lastTime;

  if (timeDiff > 100) {
    // Calcula la diferencia de pulsos
    long encoderDiff = encoderPos - lastEncoderPos;

    // Calcula la velocidad (pulsos por segundo)
    velocidad = (float)encoderDiff / timeDiff * 1000.0; // velocidad en pulsos/segundo

    // Actualiza la última posición y tiempo
    lastEncoderPos = encoderPos;
    lastTime = currentTime;

    // Imprime la velocidad y la posición en el monitor serial
    //Serial.print("Velocidad: ");
    Serial.print(velocidad);
    Serial.print("\t");
    //Serial.print(" pulsos/seg, PWM: ");
    Serial.println(pwmValue);
  }

  // Imprime el estado de los pines del encoder para diagnóstico
  // Serial.print("Encoder A: ");
  // Serial.print(digitalRead(encoderPinA));
  // Serial.print("  Encoder B: ");
  // Serial.println(digitalRead(encoderPinB));
  // delay(100);
}
