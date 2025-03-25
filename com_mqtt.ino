#include <ESP32Servo.h> // Biblioteca compatível com ESP32 baixar na 
#include <LiquidCrystal.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define NUM_SAMPLES 4  // Número de amostras para média móvel
#define MAX_VARIATION 30  // Variação máxima aceitável entre leituras consecutivas (cm)
#define SERVO_PIN 14  // Pino PWM do Servo
#define TRIG_PIN 26   // Trigger do sensor ultrassônico
#define ECHO_PIN 27   // Echo do sensor ultrassônico (usar divisor de tensão)
#define ENCODER_PIN_A  32  // Pino A do encoder
#define ENCODER_PIN_B  33  // Pino B do encoder
#define BUTTON_PIN  25    // Pino do botão do encoder (sw)

// Configuração do WiFi e MQTT
const char* ssid = "UFES_EM"; // Substitua pelo nome da sua rede WiFi
const char* password = "12345677"; // Substitua pela senha do WiFi
const char* mqtt_broker = "broker.emqx.io";  // Servidor MQTT público
const int mqtt_port = 1883;                      // Porta padrão MQTT
const char* mqtt_topic = "esp32/teste";          // Tópico onde os dados serão publicados

WiFiClient espClient;
PubSubClient client(espClient);

// Servo
Servo myServo;

// LCD
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

// Variáveis do controlador PID
double P, I, D;
double setpoint = 22.0;  
double PID;
double Kp = 10; // 25
double Ki = 3;
double Kd = 15; // 22
double lastError = 0;
double timeNow = 0;
double lastTime = 0;
const int period = 100;
double integral=0;

// Configuração do Encoder
volatile double encoderValue = 0;  // Variável para armazenar o valor do encoder
int lastEncoded = 0; // Variável para armazenar o estado anterior do encoder
unsigned long lastChangeTime = 0; // Variável para armazenar o último tempo de mudança
int sw_k = 0;
int cont = 0;
bool lastButtonState = LOW;  

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);

  //  Timeout para evitar que WiFi trave o ESP32
  Serial.print("Conectando ao WiFi...");
  WiFi.begin(ssid, password);
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (millis() - wifiStartTime > 10000) {  // 10 segundos de timeout
      Serial.println("\nFalha ao conectar ao WiFi! Continuando sem WiFi.");
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
  }

  //  Conexão MQTT com timeout
  client.setServer(mqtt_broker, mqtt_port);
  unsigned long mqttStartTime = millis();
  while (!client.connected()) {
    Serial.println("Conectando ao MQTT...");
    if (client.connect("ESP32_Client")) {
      Serial.println("Conectado ao MQTT!");
    } else {
      Serial.print("Falha na conexão MQTT, estado: ");
      Serial.println(client.state());
      if (millis() - mqttStartTime > 5000) {  // 5 segundos de timeout
        Serial.println("Falha no MQTT. Continuando sem MQTT.");
        break;
      }
      delay(2000);
    }
  }

  // Configuração do Servo
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
  if (!client.connected()) {
    reconnectMQTT(); // Tenta reconectar ao MQTT caso a conexão tenha caído
  }

  client.loop(); // Mantém a conexão ativa

  if (millis() > timeNow + period){
    timeNow = millis();
    float distance = filteredDistance();
    updatePIDValues();
  
    // Cálculo do erro
    double error = setpoint - distance;
    double vel = (error - lastError) / (period / 1000.0);
    // Cálculo do PID
    P = Kp * error;
    integral += error * (period / 1000.0);
    I = Ki * integral;
    if(abs(error) < 8){
      integral = 0;
      I = 0;
    }
    D = Kd * vel;
    lastError = error;

    PID = P + I + D;

    Serial.print("PID: ");
    Serial.print(PID*0.2 +1480);
    PID = constrain(PID*0.2 + 1485, 1300, 1600);
    
    myServo.writeMicroseconds(PID);

    Serial.print(" | I: ");
    Serial.print(I);
    Serial.print(" | Dist: ");
    Serial.println(distance);

    printValues(Kp, Kd, Ki);

    // Enviar dados via MQTT
    sendMQTTData(vel, error, P, I, D);
  }
}

// Envio de dados via MQTT
void sendMQTTData(float posicao, float erro, float p, float i, float d) {
  char payload[100];
  snprintf(payload, sizeof(payload), 
           "{\"posicao\":%.2f, \"erro\":%.2f, \"P\":%.2f, \"I\":%.2f, \"D\":%.2f}", 
           posicao, erro, p, i, d);
  client.publish(mqtt_topic, payload);
}

// Função de interrupção para atualizar o valor do encoder
void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue = encoderValue + 1;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue = encoderValue - 1;

  lastEncoded = encoded;
  lastChangeTime = millis();
}

void printValues(float kp, float kd, float ki) {
  char buffer1[16], buffer2[16];
  snprintf(buffer1, sizeof(buffer1), "Kp=%.1f Kd=%.1f", kp, kd);
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
  static float readings[NUM_SAMPLES] = {0};
  static int index = 0;
  static float lastValid = 20;
  float newReading = 0;
  
  do {
   newReading = getDistance();
  } while(newReading > 50 || newReading < 0);

  if (abs(newReading - lastValid) > MAX_VARIATION) {
    return lastValid;
  }

  readings[index] = newReading;
  index = (index + 1) % NUM_SAMPLES;

  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += readings[i];
  }
  lastValid = sum / float(NUM_SAMPLES);

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

// Conectar ao MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Conectando ao MQTT...");
    if (client.connect("ESP32_Client")) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente...");
      delay(2000);
    }
  }
}
