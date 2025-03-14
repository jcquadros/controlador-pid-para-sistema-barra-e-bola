#include <ESP32Servo.h> // Biblioteca compatível com ESP32 baixar na 
#include <LiquidCrystal.h>
#include <Ultrasonic.h>

#define pinServo 14  // Pino PWM do Servo
#define trigPin 26   // Trigger do sensor ultrassônico
#define echoPin 27   // Echo do sensor ultrassônico (usar divisor de tensão)
#define encoderPinA  32  // Pino A do encoder
#define encoderPinB  33  // Pino B do encoder
#define buttonPin  25    // Pino do botão do encoder (sw)

// Sensor
HC_SR04 sensor1(26,27);

// Servo
Servo myservo;

// LCD
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

// Variáveis do controlador PID
double setpoint = 20.0;  
double PID;
double Kp = 3.5;
double Ki = 0.2;
double Kd = 2.0;
double lastError = 0;
double timeNow = 0;
double lastTime = 0;
int period = 50;
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
  myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(pinServo, 500, 2400);
  myservo.write(125);

  // Configuração do sensor ultrassônico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Configuração do Encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  lastTime = timeNow = millis();  // Inicia o tempo do PID
}

void loop() {
  if (millis() > timeNow + period){
    timeNow = millis();
    float distance = sensor1.distance();
    //Serial.println(distance);
    updatePIDValues();
  
    // Cálculo do erro
    double error = setpoint - distance;
    
    // Cálculo PID
    double P = Kp * error;
    double I = (-3 < error && error < 3) ? I + (Ki * error): 0;
    double D = Kd * ((error - lastError) / period);
    lastError = error;
    PID = P + I + D;

    // Mapeia o valor do PID para um intervalo de aproximadamente 60 graus
    PID = map(PID, -150, 150, 60, 120); // Ajustado para permitir 60° de rotação

    // Limita o valor do PID dentro do intervalo desejado
    if (PID < 60) { PID = 60; }
    if (PID > 120) { PID = 120; }

// Aplica o valor ao servo
myservo.write(PID);


    // Aplica o valor ao servo
    myservo.write(PID);
    Serial.println(PID);
  
    printValues(Kp, Kd, Ki);
  }
}

// Função de interrupção para atualizar o valor do encoder
void updateEncoder() {
  int MSB = digitalRead(encoderPinA);  // Lê o pino A
  int LSB = digitalRead(encoderPinB);  // Lê o pino B

  int encoded = (MSB << 1) | LSB;  // Combina os dois pinos em um número
  int sum = (lastEncoded << 2) | encoded;  // Combina o estado anterior e atual

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue = encoderValue + 1;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue = encoderValue - 1;

  lastEncoded = encoded;  // Atualiza o estado anterior
  lastChangeTime = millis();  // Atualiza o tempo da última mudança
}

void printValues(float k, float kp, float ki) {
  char buffer1[16], buffer2[16];
  snprintf(buffer1, sizeof(buffer1), "K=%.2f Kp=%.2f", k, kp);
  snprintf(buffer2, sizeof(buffer2), "Ki=%.2f", ki);
  
  lcd.setCursor(0, 0);
  lcd.print(buffer1);
  lcd.setCursor(0, 1);
  lcd.print(buffer2);
}

void updatePIDValues() {
  bool currentButtonState = digitalRead(buttonPin);
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
    Kp = constrain(Kp + encoderValue * 0.1, 0.5, 10.0);
  } else if (sw_k == 1) {
    Kd = constrain(Kd + encoderValue * 0.01, 0.001, 2.0);
  } else {
    Ki = constrain(Ki + encoderValue * 0.05, 0.01, 5.0);
  }

  // Reseta o valor do encoder após cada leitura
  encoderValue = 0;
}
