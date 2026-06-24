#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- LIBRERÍAS PARA ACTUALIZACIÓN GITHUB ---
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>

const char *SSID = "Red_bombas", *PASSWORD = "Red@bombas"; 
const char *MQTT_HOST = "mqtt.tip.us-e1.tago.io", *MQTT_USER = "92cb5025", *MQTT_PASS = "ae7aa089", *SERIAL_N = "esp32_futuro_dreams";

// --- CONFIGURACIÓN GITHUB ---
const char* URL_VERSION = "https://raw.githubusercontent.com/mmar05minchel/bombas/main/version.txt";
const char* URL_BASE = "https://raw.githubusercontent.com/mmar05minchel/bombas/main/";

Preferences prefs;
String nombreArchivoActual;

// --- DECLARACIÓN EXPLÍCITA DE PUERTOS I2C (ACELERÓMETROS) ---
const int pinSDA1 = 17, pinSCL1 = 5;  // Bus 1: MPU Motor 1
const int pinSDA2 = 21, pinSCL2 = 19; // Bus 2: MPU Motor 2

// --- ASIGNACIÓN DE PINES (ADC1 ÚNICAMENTE) ---
const int pinTC1 = 32, pinTC2 = 33, pinTC3 = 35; 
const int pinPresion = 39;        
const int pinDS18B20 = 23, pinInundacion = 14, LED_INT = 2;

// --- FACTORES DE CALIBRACIÓN ---
const float FACTOR_I1 = 0.0210, FACTOR_I2 = 0.0213, FACTOR_I3 = 0.0213;

// --- CALIBRACIÓN DE DOS PUNTOS PARA PRESIÓN ---
const float PRESION_BAJA = 4.5;
const float ADC_BAJO = 2824.0; 
const float PRESION_ALTA = 5.1;
const float ADC_ALTO = 3054.2; 

// Variables Globales (Valores de ambiente fijados para simulación)
float i1=0, i2=0, i3=0, v1=0, v2=0, v3=0, t1=0, t2=0, t3=0, t4=0, temp_a=15.0, hum_a=60.0, pres=0;
float i1_r=0, i2_r=0, i3_r=0, pres_r=0; 
float v1x=0, v1y=0, v1z=0, v2x=0, v2y=0, v2z=0;
int inundacion = 0;

OneWire oneWire(pinDS18B20);
DallasTemperature sensores(&oneWire);
WiFiClient espClient;
PubSubClient mqtt(espClient);
char pushTopic[64];

// Temporizadores no bloqueantes
unsigned long lastDS18B20Req = 0;
unsigned long lastMQTTAttempt = 0;
unsigned long lastWiFiAttempt = 0; 

// Temporizadores MQTT independientes
unsigned long lastMQTT_1m = 0;
unsigned long lastMQTT_5m = 0;
unsigned long lastMQTT_20m = 0;

unsigned long lastOTACheck = 0;   

// =================================================================================
// ⏱️ APARTADO DE CONFIGURACIÓN DE TIEMPO
// =================================================================================
const int HORAS_DE_ESPERA   = 1;  
const int MINUTOS_DE_ESPERA = 0;  
// =================================================================================

void verificarYActualizar() {
  Serial.println("\n[OTA] Verificando nueva versión en GitHub...");
  WiFiClientSecure client;
  client.setInsecure(); 
  client.setTimeout(15000); 
  
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(client, URL_VERSION);
  
  int httpCode = http.GET(); 
  
  if (httpCode == 200) { 
    String nuevoNombre = http.getString();
    nuevoNombre.trim();
    
    if (nuevoNombre != nombreArchivoActual) {
      Serial.println("[OTA] ¡Actualización encontrada! Descargando...");
      httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      t_httpUpdate_return ret = httpUpdate.update(client, String(URL_BASE) + nuevoNombre);
      
      if (ret == HTTP_UPDATE_OK) {
        prefs.putString("firmware", nuevoNombre);
        ESP.restart();
      }
    } else {
      Serial.println("[OTA] Firmware actualizado.");
    }
  } else {
    Serial.println("[OTA] Error al consultar GitHub.");
  }
  http.end();
}

void calcRMS_I(int pin, float factor, float &raw, float &real) {
  long sum = 0; 
  const int num_samples = 200; 
  int v[num_samples];
  
  for(int i=0; i<num_samples; i++) { 
    v[i] = analogRead(pin); 
    sum += v[i]; 
    delayMicroseconds(150); 
  }
  
  float mean = sum / (float)num_samples; 
  double sqSum = 0;
  for(int i=0; i<num_samples; i++) sqSum += sq(v[i] - mean);
  
  raw = sqrt(sqSum / (float)num_samples);
  real = raw * factor; 
  if(real <= 0.40) real = 0.0;
}

// --- FUNCIÓN DE LECTURA OPTIMIZADA ANTI-CONGELAMIENTO ---
void leerMPU(TwoWire &bus, int16_t ox, int16_t oy, int16_t oz, float &x, float &y, float &z) {
  bus.beginTransmission(0x68); 
  bus.write(0x3B); 
  
  // Si el dispositivo no responde en el bus, salimos de inmediato para evitar el cuelgue
  if (bus.endTransmission(false) != 0) {
    x = 0; y = 0; z = 0;
    return; 
  }
  
  // Solo si la verificación previa fue exitosa, solicitamos los bytes
  if (bus.requestFrom(0x68, 6) >= 6) {
    int16_t rawX = (bus.read() << 8) | bus.read();
    int16_t rawY = (bus.read() << 8) | bus.read();
    int16_t rawZ = (bus.read() << 8) | bus.read();
    
    x = (rawX - ox) / 100.0;
    y = (rawY - oy) / 100.0;
    z = (rawZ - oz) / 100.0;
  } else {
    x = 0; y = 0; z = 0;
  }
}

void reinitMPU(TwoWire &bus) {
  bus.beginTransmission(0x68); bus.write(0x6B); bus.write(0); bus.endTransmission();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_INT, OUTPUT); 
  pinMode(pinPresion, ANALOG); 
  pinMode(pinInundacion, INPUT_PULLUP);
  
  prefs.begin("sistema", false);
  nombreArchivoActual = prefs.getString("firmware", "bomba_1.0.bin");

  int pines[] = {pinTC1, pinTC2, pinTC3};
  for(int p : pines) pinMode(p, ANALOG);
  analogSetAttenuation(ADC_11db);

  // --- INICIALIZACIÓN UTILIZANDO LAS CONSTANTES DECLARADAS ---
  Wire.begin(pinSDA1, pinSCL1, 100000); 
  Wire.setTimeOut(150); 
  Wire1.begin(pinSDA2, pinSCL2, 100000); 
  Wire1.setTimeOut(150);
  
  reinitMPU(Wire);
  reinitMPU(Wire1);

  sensores.begin();
  sensores.setWaitForConversion(false); 

  snprintf(pushTopic, sizeof(pushTopic), "$tip/%s/push", SERIAL_N);
  mqtt.setServer(MQTT_HOST, 1883); mqtt.setBufferSize(1024);
  
  Serial.println("\n[WIFI] Intentando conectar a la red...");
  WiFi.begin(SSID, PASSWORD);
  
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) { 
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Conectado exitosamente.");
  } else {
    Serial.println("\n[WIFI] Red no encontrada. Iniciando sensores en modo local (reintento en 5 min).");
  }
  
  lastOTACheck = millis(); 
  lastWiFiAttempt = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. GESTIÓN DE CONEXIÓN WI-FI Y MQTT
  if (WiFi.status() != WL_CONNECTED) {
    if (currentMillis - lastWiFiAttempt >= 300000) {
      lastWiFiAttempt = currentMillis;
      Serial.println("\n[WIFI] Han pasado 5 minutos. Reintentando conexión a Wi-Fi...");
      WiFi.disconnect();
      WiFi.begin(SSID, PASSWORD);
    }
  } else {
    if (!mqtt.connected()) { 
      if (currentMillis - lastMQTTAttempt > 5000) { 
        lastMQTTAttempt = currentMillis;
        mqtt.connect(SERIAL_N, MQTT_USER, MQTT_PASS);
      }
    } else {
      mqtt.loop();
    }
  }

  // 2. LECTURAS DE ACELERÓMETROS CON PROTECCIÓN
  leerMPU(Wire, -300, 16000, 908, v1x, v1y, v1z);
  leerMPU(Wire1, -900, 16172, 2004, v2x, v2y, v2z);

  calcRMS_I(pinTC1, FACTOR_I1, i1_r, i1); 
  calcRMS_I(pinTC2, FACTOR_I2, i2_r, i2); 
  calcRMS_I(pinTC3, FACTOR_I3, i3_r, i3);
  
  v1 = random(2254, 2347) / 10.0;
  v2 = random(2254, 2347) / 10.0;
  v3 = random(2254, 2347) / 10.0; 

  // 3. DS18B20 ASÍNCRONO
  if (currentMillis - lastDS18B20Req > 1000) {
    lastDS18B20Req = currentMillis;
    t1 = max(0.0f, sensores.getTempCByIndex(0)); t2 = max(0.0f, sensores.getTempCByIndex(1));
    t3 = max(0.0f, sensores.getTempCByIndex(2)); t4 = max(0.0f, sensores.getTempCByIndex(3));
    sensores.requestTemperatures(); 
  }

  // 4. LÓGICA DE PRESIÓN EN PIN 39
  long sumaADC = 0;
  for (int i = 0; i < 20; i++) {
    sumaADC += analogRead(pinPresion);
    delay(10);
  }
  pres_r = sumaADC / 20.0; 

  float p_calc = PRESION_BAJA + (pres_r - ADC_BAJO) * ((PRESION_ALTA - PRESION_BAJA) / (ADC_ALTO - ADC_BAJO));
  if (p_calc < 0.0) p_calc = 0.0;
  if (p_calc > 10.0) p_calc = 10.0;
  pres = p_calc;

  inundacion = digitalRead(pinInundacion);

  // --- IMPRESIÓN EN MONITOR SERIE COMPLETA (CADA 2 SEG) ---
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint > 2000) {
    lastPrint = currentMillis;
    Serial.println("\n================ RESUMEN DE TELEMETRÍA ================");
    Serial.printf("ESTADO WI-FI | %s\n", (WiFi.status() == WL_CONNECTED) ? "Conectado" : "DESCONECTADO (Guardando en local)");
    Serial.println("-------------------------------------------------------");
    Serial.printf("VOLTAJE      | (Simulados a ~230V) V1: %5.1f V | V2: %5.1f V | V3: %5.1f V\n", v1, v2, v3);
    Serial.printf("CORRIENTE    | I1 Crudo: %6.1f -> %5.2f A | I2 Crudo: %6.1f -> %5.2f A | I3 Crudo: %6.1f -> %5.2f A\n", i1_r, i1, i2_r, i2, i3_r, i3);
    Serial.printf("PRESIÓN      | P1 Crudo: %6.1f -> %5.2f Bar\n", pres_r, pres);
    Serial.println("-------------------------------------------------------");
    Serial.printf("VIBRACIÓN 1  | X: %5.2f | Y: %5.2f | Z: %5.2f\n", v1x, v1y, v1z);
    Serial.printf("VIBRACIÓN 2  | X: %5.2f | Y: %5.2f | Z: %5.2f\n", v2x, v2y, v2z);
    Serial.printf("TEMP MOTORES | T1: %5.1f C | T2: %5.1f C | T3: %5.1f C | T4: %5.1f C\n", t1, t2, t3, t4);
    Serial.printf("AMBIENTE     | Temp: %5.1f C | Humedad: %5.1f %%\n", temp_a, hum_a);
    Serial.printf("INUNDACIÓN   | Estado Sensor (Pin 14): %d\n", inundacion);
    Serial.println("=======================================================\n");
  }

  // 5. PUBLICACIÓN MQTT CON TRANSMISIÓN VISUAL (DESTELLO LED)
  if (mqtt.connected()) {
    
    // --- ENVÍO DE 1 MINUTO ---
    if (currentMillis - lastMQTT_1m > 60000) {
      lastMQTT_1m = currentMillis;
      char pl_1m[512]; 
      snprintf(pl_1m, sizeof(pl_1m), "[v1:=%.1f#V;v2:=%.1f#V;v3:=%.1f#V;"
               "i1:=%.2f#A;i2:=%.2f#A;i3:=%.2f#A;pres:=%.2f#Bar;"
               "vib1_x:=%.1f#mm/s;vib1_y:=%.1f#mm/s;vib1_z:=%.1f#mm/s;"
               "vib2_x:=%.1f#mm/s;vib2_y:=%.1f#mm/s;vib2_z:=%.1f#mm/s]", 
               v1, v2, v3, i1, i2, i3, pres, v1x, v1y, v1z, v2x, v2y, v2z); 
      
      digitalWrite(LED_INT, HIGH);
      mqtt.publish(pushTopic, pl_1m);
      delay(50);
      digitalWrite(LED_INT, LOW);
    }

    // --- ENVÍO DE 5 MINUTOS ---
    if (currentMillis - lastMQTT_5m > 300000) {
      lastMQTT_5m = currentMillis;
      char pl_5m[256]; 
      snprintf(pl_5m, sizeof(pl_5m), "[t1:=%.1f#C;t2:=%.1f#C;t3:=%.1f#C;t4:=%.1f#C;inund:=%d]", 
               t1, t2, t3, t4, inundacion); 
      
      digitalWrite(LED_INT, HIGH);
      mqtt.publish(pushTopic, pl_5m);
      delay(50);
      digitalWrite(LED_INT, LOW);
    }

    // --- ENVÍO DE 20 MINUTOS ---
    if (currentMillis - lastMQTT_20m > 1200000) {
      lastMQTT_20m = currentMillis;
      char pl_20m[128]; 
      snprintf(pl_20m, sizeof(pl_20m), "[temp_a:=%.1f#C;hum_a:=%.1f#%%]", temp_a, hum_a); 
      
      digitalWrite(LED_INT, HIGH);
      mqtt.publish(pushTopic, pl_20m);
      delay(50);
      digitalWrite(LED_INT, LOW);
    }
  }

  // 6. LÓGICA DE ACTUALIZACIÓN GITHUB
  unsigned long intervaloOTA = (HORAS_DE_ESPERA * 3600000UL) + (MINUTOS_DE_ESPERA * 60000UL);
  if (currentMillis - lastOTACheck >= intervaloOTA) {
    lastOTACheck = currentMillis; 
    if (WiFi.status() == WL_CONNECTED) {
      verificarYActualizar();
    } else {
      Serial.println("[OTA] Wi-Fi desconectado. Se omite revisión de actualización.");
    }
  }
}