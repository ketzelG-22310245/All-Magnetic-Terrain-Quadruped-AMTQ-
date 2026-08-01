#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* ssid = "Redmi"; 
const char* password = "12345678";

// Configuración UDP
WiFiUDP udp;
const int udpPort = 5000;
const int NUM_SERVOS = 12; 
const int NUM_SENALES = 16; // 12 motores + 4 imanes

// --- PLACAS PCA9685 ---
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40); // LADO IZQUIERDO
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41); // LADO DERECHO

// Pines de I2C en la ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// --- PINES DE LOS 4 RELÉS (CORREGIDOS POR SOFTWARE) ---
const int PIN_IMAN_ID = 4;   // Cambiado de 17 a 4 (Ahora controla físicamente la pata ID)
const int PIN_IMAN_IT = 16;  // Se queda en 16 (Este estaba perfecto)
const int PIN_IMAN_DD = 18;  // Cambiado de 4 a 17 (Ahora controla físicamente la pata DD)
const int PIN_IMAN_DT = 17;  // Se queda en 18 (Monitorea su comportamiento tras el cruce) 

// LEDs indicadores
const int statusLed = 19;   
const int dataLed = 2;     

// Límites de tus servos Feetech
#define SERVOMIN  110 
#define SERVOMAX  510 

// --- ARRAY DE INVERSIÓN GENERAL DEFINITIVO ---
bool invertirServo[NUM_SERVOS] = {
  false, true,  true,  // 0-2:  Pata Izquierda Delantera (ID)
  false, true,  true,  // 3-5:  Pata Izquierda Trasera (IT)   
  true,  false, false, // 6-8:  Pata Derecha Delantera (DD)   
  true,  false, false  // 9-11: Pata Derecha Trasera (DT)     
};

bool primeraVez = true; 
unsigned long ultimaVezRecibido = 0; 

void setup() {
  Serial.begin(115200);
  
  pinMode(statusLed, OUTPUT);
  pinMode(dataLed, OUTPUT);
  
  pinMode(PIN_IMAN_ID, OUTPUT);
  pinMode(PIN_IMAN_IT, OUTPUT);
  pinMode(PIN_IMAN_DD, OUTPUT);
  pinMode(PIN_IMAN_DT, OUTPUT);
  
  // ARRANQUE SEGURO (JUMPERS EN HIGH + CABLES EN NC):
  // Mandamos HIGH al arrancar. Con el jumper en HIGH, esto activa el relé,
  // abriendo el circuito NC y asegurando que los imanes inicien APAGADOS.
  digitalWrite(PIN_IMAN_ID, HIGH);
  digitalWrite(PIN_IMAN_IT, HIGH);
  digitalWrite(PIN_IMAN_DD, HIGH);
  digitalWrite(PIN_IMAN_DT, HIGH);

  Serial.println("\n=== CONFIGURACIÓN: JUMPERS EN HIGH + CABLES EN NC ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  
  pca1.begin();
  pca1.setOscillatorFrequency(27000000);
  pca1.setPWMFreq(50);

  pca2.begin();
  pca2.setOscillatorFrequency(27000000);
  pca2.setPWMFreq(50);

  Serial.print("Conectando a la red WiFi: ");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(statusLed, !digitalRead(statusLed)); 
  }

  // === AQUÍ ESTÁ CORREGIDO: IMPRESIÓN DE LA DIRECCIÓN IP ===
  Serial.println("\n¡Conectado exitosamente!");
  Serial.print("Dirección IP asignada a la ESP32: ");
  Serial.println(WiFi.localIP()); // <-- Esta es la línea mágica que faltaba
  Serial.print("Puerto UDP abierto: ");
  Serial.println(udpPort);
  Serial.println("==================================================\n");
  
  digitalWrite(statusLed, HIGH);  
  udp.begin(udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    digitalWrite(dataLed, HIGH); 
    
    uint8_t packetBuffer[NUM_SENALES]; 
    udp.read(packetBuffer, NUM_SENALES); 
    
    ultimaVezRecibido = millis(); 

    // === LÓGICA ESTRICTA PARA JUMPERS EN HIGH + CABLES EN NC ===
    // Si Simulink manda 1 -> El pin baja a LOW. El relé se libera (apaga), regresa a NC -> PRENDE IMÁN.
    // Si Simulink manda 0 -> El pin sube a HIGH. El relé se activa, abre NC -> APAGA IMÁN.
    
    digitalWrite(PIN_IMAN_ID, packetBuffer[12] == 1 ? LOW : HIGH);
    digitalWrite(PIN_IMAN_IT, packetBuffer[13] == 1 ? LOW : HIGH);
    digitalWrite(PIN_IMAN_DD, packetBuffer[14] == 1 ? LOW : HIGH);
    digitalWrite(PIN_IMAN_DT, packetBuffer[15] == 1 ? LOW : HIGH);

    // === IMPRESIÓN DE DIAGNÓSTICO ===
    Serial.print("IMANES -> ");
    Serial.print("ID[12]: "); Serial.print(packetBuffer[12] == 1 ? "ON (1)" : "OFF(0)");
    Serial.print(" | IT[13]: "); Serial.print(packetBuffer[13] == 1 ? "ON (1)" : "OFF(0)");
    Serial.print(" | DD[14]: "); Serial.print(packetBuffer[14] == 1 ? "ON (1)" : "OFF(0)");
    Serial.print(" | DT[15]: "); Serial.print(packetBuffer[15] == 1 ? "ON (1)" : "OFF(0)");
    Serial.print(" | ");

    int angulosObjetivo[NUM_SERVOS];
    for(int i = 0; i < NUM_SERVOS; i++) {
      angulosObjetivo[i] = packetBuffer[i];
      if (invertirServo[i]) angulosObjetivo[i] = 180 - angulosObjetivo[i];
      angulosObjetivo[i] = constrain(angulosObjetivo[i], 0, 180);
    }

    // Arranque suave
    if (primeraVez) {
      int angulosActuales[NUM_SERVOS];
      for(int i = 0; i < NUM_SERVOS; i++) angulosActuales[i] = 90;

      bool alcanzado = false;
      while (!alcanzado) {
        alcanzado = true; 
        for(int i = 0; i < NUM_SERVOS; i++) {
          if (angulosActuales[i] < angulosObjetivo[i]) { angulosActuales[i]++; alcanzado = false; } 
          else if (angulosActuales[i] > angulosObjetivo[i]) { angulosActuales[i]--; alcanzado = false; }
          
          int pulso = map(angulosActuales[i], 0, 180, SERVOMIN, SERVOMAX);
          if (i < 6) pca1.setPWM(i, 0, pulso);
          else pca2.setPWM(i - 6, 0, pulso);
        }
        delay(10); 
      }
      primeraVez = false; 
    }

    // Control normal de servos
    for(int i = 0; i < NUM_SERVOS; i++) {
      int pulso = map(angulosObjetivo[i], 0, 180, SERVOMIN, SERVOMAX);
      if (i < 6) pca1.setPWM(i, 0, pulso);
      else pca2.setPWM(i - 6, 0, pulso);
    }

    digitalWrite(dataLed, LOW); 
  }

  // === CORTAFUEGOS DE SEGURIDAD ===
  if (millis() - ultimaVezRecibido > 500) {
    digitalWrite(PIN_IMAN_ID, HIGH);
    digitalWrite(PIN_IMAN_IT, HIGH);
    digitalWrite(PIN_IMAN_DD, HIGH);
    digitalWrite(PIN_IMAN_DT, HIGH);
  }
}
