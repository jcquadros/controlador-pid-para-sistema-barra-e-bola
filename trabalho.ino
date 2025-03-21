#include <ESP32Servo.h> // Biblioteca compatível com ESP32 baixar na 
#include <LiquidCrystal.h>

#define NUM_SAMPLES 4  // Número de amostras para média móvel
#define MAX_VARIATION 40  // Variação máxima aceitável entre leituras consecutivas (cm)
#define SERVO_PIN 14  // Pino PWM do Servo
#define TRIG_PIN 26   // Trigger do sensor ultrassônico
#define ECHO_PIN 27   // Echo do sensor ultrassônico (usar divisor de tensão)
#define ENCODER_PIN_A  32  // Pino A do encoder
#define ENCODER_PIN_B  33  // Pino B do encoder
#define BUTTON_PIN  25    // Pino do botão do encoder (sw)
// Servo
Servo myServo;

// LCD
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

// Variáveis do controlador PID
double P,I,D;
double setpoint = 19.0;  
double PID;
double Kp = 2;
double Ki = 0.01;
double Kd = 2.0;
double lastError = 0;
double timeNow = 0;
double lastTime = 0;
const int period = 100;
double integral;
// Configuração do Encoder
volatile double encoderValue = 0;  // Variável para armazenar o valor do encoder
int lastEncoded = 0;            // Variável para armazenar o estado anterior do encoder
unsigned long lastChangeTime = 0;  // Variável para armazenar o último tempo de mudança
int sw_k = 0;
int cont = 0;
bool lastButtonState = LOW;  

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);

  // Condiguracao do servo
	myServo.attach(SERVO_PIN);

  // Configuração do sensor ultrassônico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configuração do Encoder
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  
  lastTime = timeNow = millis();  // Inicia o tempo do PID
}

void loop() {
  if (millis() > timeNow + period){
    timeNow = millis();
    float distance = filteredDistance();
    updatePIDValues();
  
    // Cálculo do erro
    double error = setpoint - distance;
    double vel = (error - lastError) / (period / 1000.0);
    // Cálculo do PID
    P = Kp * error;
    I = (abs(error)>1 && abs(error)<5) ? I + (Ki * error) : 0;
    D = Kd * vel;  // Derivativo
    lastError = error;

    PID = P + I + D;
    //int servoPulse = map(angle, 0, 45, SERVO_MIN_US, SERVO_MAX_US);

    // Escreve no servo com precisão em microsegundos
    Serial.print("PID: ");
    Serial.print(PID +1500);
    PID = constrain(PID + 1500, 1300, 1600); // limita entre 1300 e 1600 microsegundos
    
    myServo.writeMicroseconds(PID);

    Serial.print(" | I: ");
    Serial.print(I);
    Serial.print(" | Dist: ");
    Serial.println(distance);

    printValues(Kp, Kd, Ki);
  }
}

// Função de interrupção para atualizar o valor do encoder
void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);  // Lê o pino A
  int LSB = digitalRead(ENCODER_PIN_B);  // Lê o pino B

  int encoded = (MSB << 1) | LSB;  // Combina os dois pinos em um número
  int sum = (lastEncoded << 2) | encoded;  // Combina o estado anterior e atual

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue = encoderValue + 1;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue = encoderValue - 1;

  lastEncoded = encoded;  // Atualiza o estado anterior
  lastChangeTime = millis();  // Atualiza o tempo da última mudança
}

void printValues(float kp, float kd, float ki) {
  char buffer1[16], buffer2[16];
  snprintf(buffer1, sizeof(buffer1), "Kp=%.2f Kd=%.2f", kp, kd);
  snprintf(buffer2, sizeof(buffer2), "Ki=%.2f", ki);
  
  lcd.setCursor(0, 0);
  lcd.print(buffer1);
  lcd.setCursor(0, 1);
  lcd.print(buffer2);
}

void updatePIDValues() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  // Verifica se o botão do encoder foi pressionado
  if (cont == 150) {
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      sw_k += 1;
      if (sw_k == 3) sw_k = 0;
    }
    lastButtonState = currentButtonState; // pra evitar mais de uma pressionada
  } else {
    cont += 1;
  }

  // Atualizando valores de K em uma escala diferente pra cada um
  if (sw_k == 0) {
    Kp = constrain(Kp + encoderValue * 1.0, 0, 100);
  } else if (sw_k == 1) {
    Kd = constrain(Kd + encoderValue * 1.0, 0, 100);
  } else {
    Ki = constrain(Ki + encoderValue * 0.01, 0.00, 5.0);
  }

  // Reseta o valor do encoder após cada leitura
  encoderValue = 0;
}

float filteredDistance() {
  static float readings[NUM_SAMPLES] = {0};  // Buffer de leituras
  static int index = 0;
  static float lastValid = 20;  // Última distância válida (inicialmente 20 cm)
  float newReading = 0;
  
  do {
   newReading = getDistance();  // Obtém nova leitura do sensor
  } while(newReading > 50 || newReading < 0);

  // Descartar leituras anômalas (muito diferentes da última válida)
  if (abs(newReading - lastValid) > MAX_VARIATION) {
    return lastValid;  // Mantém a última leitura válida
  }

  // Atualiza buffer circular
  readings[index] = newReading;
  index = (index + 1) % NUM_SAMPLES;

  // Calcula a média das leituras armazenadas
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += readings[i];
  }
  lastValid = sum / float(NUM_SAMPLES);  // Atualiza última leitura válida

  return int(lastValid);
}


float getDistance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  return int(pulseIn(ECHO_PIN, HIGH) * 0.01715); 
}
