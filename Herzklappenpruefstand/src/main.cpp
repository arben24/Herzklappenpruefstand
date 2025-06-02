


#include <Adafruit_ADS1X15.h>
#include "HX711.h"
#include <ODriveUART.h>

// ADS1015 (12-Bit)
Adafruit_ADS1015 ads;

// Pindefinitionen
int relayPin = 12;    // Relais
int flowPin = 13;     // Flow-Sensor (Puls-Ausgang)

// HX711-Pins (Wägezellen)
// Sensor 1
int DT1 = 25;
int SCK1 = 26;
// Sensor 2
int DT2 = 27;
int SCK2 = 14;

HX711 scale1;
HX711 scale2;

float hx711_1_value = 0.0;
float hx711_2_value = 0.0;

// ODrive Motor
HardwareSerial& odrive_serial = Serial2;
ODriveUART odrive(odrive_serial);
int baudrate = 115200;
float position = 0;   // Aktuelle Zielposition (in Bogenmaß)
float motorSpeed = 0; // Motor-Geschwindigkeit (rad/s)
  
// Flow Sensor Variablen
volatile int flow_count = 0;     // Wird in der ISR erhöht
float flowRate = 0.0;            // Durchfluss in L/min
unsigned long previousFlowMillis = 0;

void flow_ISR() {
  flow_count++;
}

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  Serial.println("Starte Serial Monitor Steuerung");
  
  // Relais initialisieren
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  Serial.println("Relay initialisiert");

  // Flow-Sensor initialisieren (Interrupt an flowPin)
  pinMode(flowPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin), flow_ISR, RISING);
  
  // ADS1015 initialisieren
  if (!ads.begin()) {
    Serial.println("Fehler: ADS1015 nicht gefunden!");
    while (1) { delay(100); }
  }
  
  // HX711 Sensoren initialisieren
  Serial.println("Initialisiere HX711 Sensor 1...");
  scale1.begin(DT1, SCK1);
  scale1.set_scale(101.158);        // Kalibrierungsfaktor anpassen!
  scale1.set_offset(-59685);      // Offset anpassen!
  Serial.println("Sensor 1 bereit");
  
  Serial.println("Initialisiere HX711 Sensor 2...");
  scale2.begin(DT2, SCK2);
  scale2.set_scale(99.44);        // Kalibrierungsfaktor anpassen!
  scale2.set_offset(391202);      // Offset anpassen!
  Serial.println("Sensor 2 bereit");
  
  // ODrive initialisieren
  odrive_serial.begin(baudrate, SERIAL_8N1, 16, 17);
  Serial.println("Warte auf ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  Serial.println("ODrive gefunden");
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  Serial.println("ODrive läuft!");
  // Starte im Idle-Modus und lese die aktuelle Position
  odrive.setState(AXIS_STATE_IDLE);
  position = odrive.getPosition();

  // Gib verfügbare Befehle aus
  Serial.println("Verfügbare Befehle:");
  Serial.println("  relay on         -- schaltet das Relais ein");
  Serial.println("  relay off        -- schaltet das Relais aus");
  Serial.println("  motor pos X      -- fährt den Motor auf Position X (Grad)");
  Serial.println("  motor speed X    -- setzt die Motor-Geschwindigkeit (rad/s)");
  Serial.println("  motor idle       -- versetzt den Motor in den Idle-Modus");
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("relay on")) {
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay eingeschaltet");
    }
    else if (cmd.equalsIgnoreCase("relay off")) {
      digitalWrite(relayPin, LOW);
      Serial.println("Relay ausgeschaltet");
    }
    else if (cmd.startsWith("motor pos ")) {
      // Beispiel: "motor pos 90" (Grad)
      String posStr = cmd.substring(10);  // Nach "motor pos "
      int posDeg = posStr.toInt();
      position = (posDeg * 6.28) / 360;     // Grad in Bogenmaß
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      odrive.setPosition(position);
      Serial.print("Motorposition auf ");
      Serial.print(position, 4);
      Serial.println(" rad gesetzt");
    }
    else if (cmd.startsWith("motor speed ")) {
      // Beispiel: "motor speed 0.5" (rad/s)
      String speedStr = cmd.substring(12); // Nach "motor speed "
      motorSpeed = speedStr.toFloat();
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      Serial.print("Motor-Geschwindigkeit auf ");
      Serial.print(motorSpeed, 4);
      Serial.println(" rad/s gesetzt");
    }
    else if (cmd.equalsIgnoreCase("motor idle")) {
      odrive.clearErrors();
      odrive.setState(AXIS_STATE_IDLE);
      Serial.println("Motor in Idle-Modus");
    }
    else {
      Serial.print("Unbekannter Befehl: ");
      Serial.println(cmd);
      Serial.println("Befehle: relay on/off, motor pos X, motor speed X, motor idle");
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Sensorwerte (ADC) alle 1 Sekunde auslesen und ausgeben
  static unsigned long prevADCMillis = 0;
  if (currentMillis - prevADCMillis >= 10) {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    int16_t adc1 = ads.readADC_SingleEnded(1);
    float volts0 = ads.computeVolts(adc0);
    float volts1 = ads.computeVolts(adc1);

    Serial.print(adc0);
    Serial.print(" ");
    Serial.print(volts0, 4);
    Serial.print(" ");
    Serial.print(adc1);
    Serial.print(" ");
    Serial.print(volts1, 4);
    Serial.println();
    prevADCMillis = currentMillis;
  }
  

  

  
  //Kontinuierliche Motorsteuerung:
  static unsigned long prevMotorMillis = millis();
  unsigned long currentMotorMillis = millis();
  float dtMotor = (currentMotorMillis - prevMotorMillis) / 1000.0;
  prevMotorMillis = currentMotorMillis;
  // Erhöhe (oder verringere) die Position basierend auf der eingestellten Geschwindigkeit
  position += motorSpeed * dtMotor;
  //odrive.setPosition(position);
  static unsigned long prevMotorPrint = 0;
  if (currentMillis - prevMotorPrint >= 1000) {
    Serial.print("Motorposition: ");
    Serial.print(position, 4);
    Serial.print(" rad, Motor-Geschwindigkeit: ");
    Serial.println(motorSpeed, 4);
    prevMotorPrint = currentMillis;
  }
  
  // Verarbeite serielle Befehle
  processSerialCommands();
  
  delay(1);
}






// #include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <Adafruit_ADS1X15.h>
// #include "HX711.h"
// #include <ODriveUART.h>

// // Verwende den 12-Bit ADS1015
// Adafruit_ADS1015 ads;

// // WiFi-Zugangsdaten (bitte anpassen)
// const char* ssid = "Leobots";
// const char* password = "leobots42";

// // Pindefinitionen
// int relayPin = 12;    // Relais
// int flowPin = 13;     // Flow-Sensor (Puls-Ausgang)

// // Wägezellendefinition (HX711)
// // Sensor 1
// int DT1 = 25;
// int SCK1 = 26;
// // Sensor 2
// int DT2 = 27;
// int SCK2 = 14;

// HX711 scale1;
// HX711 scale2;

// float hx711_1_value = 0.0;
// float hx711_2_value = 0.0;

// // ODrive Motor
// HardwareSerial& odrive_serial = Serial2;
// ODriveUART odrive(odrive_serial);
// int baudrate = 115200;
// float position = 0;   // Aktuelle Zielposition (in Bogenmaß)

// // Motor Speed Control (neuer kontinuierlicher Modus)
// // motorSpeed in rad/s (wenn 0, wird der Motor nicht kontinuierlich weiterbewegt)
// float motorSpeed = 0.0;
// unsigned long previousMotorMillis = 0;

// // AsyncWebServer läuft auf Port 80
// AsyncWebServer server(80);

// // Flow Sensor Variablen
// volatile int flow_count = 0;     // Anzahl der Pulse (wird in ISR erhöht)
// float flowRate = 0.0;            // Durchfluss in L/min
// unsigned long previousFlowMillis = 0;

// void flow_ISR() {
//   flow_count++;
// }

// void setup() {
//   // Serielle Schnittstelle initialisieren
//   Serial.begin(115200);
//   Serial.println("Hello!");
//   Serial.println("Lese AIN0 und AIN1 vom ADS1015");

//   // Relais initialisieren
//   pinMode(relayPin, OUTPUT);
//   digitalWrite(relayPin, LOW);  // Relais standardmäßig aus
//   Serial.println("Relay Control via Web");

//   // Flow-Sensor-Pin als Eingang definieren und Interrupt aktivieren
//   pinMode(flowPin, INPUT);
//   attachInterrupt(digitalPinToInterrupt(flowPin), flow_ISR, RISING);

//   // ADS1015 initialisieren
//   if (!ads.begin()) {
//     Serial.println("Fehler: ADS1015 konnte nicht initialisiert werden!");
//     while (1);
//   }

//   // ----- HX711 Sensor 1 initialisieren -----
//   Serial.println("HX711 Sensor 1 initialisieren...");
//   scale1.begin(DT1, SCK1);
//   scale1.set_scale(100);        // Kalibrierungsfaktor anpassen!
//   // Optional: scale1.tare();
//   scale1.set_offset(208724);       // Offset anpassen!
//   Serial.println("HX711 Sensor 1 initialisiert");

//   // ----- HX711 Sensor 2 initialisieren -----
//   Serial.println("HX711 Sensor 2 initialisieren...");
//   scale2.begin(DT2, SCK2);
//   scale2.set_scale(100);        // Kalibrierungsfaktor anpassen!
//   // Optional: scale2.tare();
//   scale2.set_offset(208724);       // Offset anpassen!
//   Serial.println("HX711 Sensor 2 initialisiert");

//   // ODrive initialisieren
//   odrive_serial.begin(baudrate, SERIAL_8N1, 16, 17);
//   Serial.println("Waiting for ODrive...");
//   while (odrive.getState() == AXIS_STATE_UNDEFINED) {
//     delay(100);
//   }
//   Serial.println("ODrive gefunden");
//   Serial.print("DC voltage: ");
//   Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
//   Serial.println("Enabling closed loop control...");
//   while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
//     odrive.clearErrors();
//     odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
//     delay(10);
//   }
//   Serial.println("ODrive running!");
//   // Startet im Idle-Modus
//   odrive.setState(AXIS_STATE_IDLE);
//   position = odrive.getPosition();

//   // WLAN-Verbindung herstellen
//   WiFi.begin(ssid, password);
//   Serial.print("Verbinde mit WiFi");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println();
//   Serial.print("Verbunden, IP-Adresse: ");
//   Serial.println(WiFi.localIP());

//   // Initialisiere Motor Timing
//   previousMotorMillis = millis();

//   // Web-Dashboard
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//     String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
//     html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
//     html += "<title>ESP32 Dashboard</title>";
//     html += "<script>";
//     // Funktion zum periodischen Abruf der Sensorwerte
//     html += "function fetchData() {";
//     html += "  fetch('/values')"
//             ".then(response => response.json())"
//             ".then(data => {";
//     html += "    document.getElementById('adc0').innerHTML = data.adc0;";
//     html += "    document.getElementById('volts0').innerHTML = data.volts0.toFixed(4);";
//     html += "    document.getElementById('adc1').innerHTML = data.adc1;";
//     html += "    document.getElementById('volts1').innerHTML = data.volts1.toFixed(4);";
//     html += "    document.getElementById('hx711_1').innerHTML = data.hx711_1.toFixed(2);";
//     html += "    document.getElementById('hx711_2').innerHTML = data.hx711_2.toFixed(2);";
//     html += "    document.getElementById('flow').innerHTML = data.flow.toFixed(2);";
//     html += "  })"
//             ".catch(err => console.error(err));";
//     html += "}";
//     html += "setInterval(fetchData, 1000);";
//     // Funktion zur Steuerung des Relais
//     html += "function setRelay(state) {";
//     html += "  fetch('/relay?state=' + state)"
//             ".then(response => response.text())"
//             ".then(result => { console.log(result); })"
//             ".catch(err => console.error(err));";
//     html += "}";
//     // Funktion zur Steuerung des Motors (Position setzen)
//     html += "function setMotor(pos) {";
//     html += "  fetch('/motor?position=' + pos)"
//             ".then(response => response.text())"
//             ".then(result => { console.log(result); })"
//             ".catch(err => console.error(err));";
//     html += "}";
//     // Funktion zur Steuerung des Motors in den Idle-Modus
//     html += "function setMotorIdle() {";
//     html += "  fetch('/motorIdle')"
//             ".then(response => response.text())"
//             ".then(result => { console.log(result); })"
//             ".catch(err => console.error(err));";
//     html += "}";
//     // Funktion zur Steuerung der Motor-Geschwindigkeit
//     html += "function setMotorSpeed(speed) {";
//     html += "  fetch('/motorspeed?speed=' + speed)"
//             ".then(response => response.text())"
//             ".then(result => { console.log(result); })"
//             ".catch(err => console.error(err));";
//     html += "}";
//     html += "</script></head><body>";
//     html += "<h1>ESP32 Dashboard</h1>";
//     html += "<h2>ADC Readings</h2>";
//     html += "<p>AIN0: <span id='adc0'>--</span> (<span id='volts0'>--</span> V)</p>";
//     html += "<p>AIN1: <span id='adc1'>--</span> (<span id='volts1'>--</span> V)</p>";
//     html += "<h2>Wägezellen</h2>";
//     html += "<p>Wägezelle 1: <span id='hx711_1'>--</span></p>";
//     html += "<p>Wägezelle 2: <span id='hx711_2'>--</span></p>";
//     html += "<h2>Flow Sensor</h2>";
//     html += "<p>Flow: <span id='flow'>--</span> L/min</p>";
//     html += "<h2>Relay Control</h2>";
//     html += "<button onclick=\"setRelay('ON')\">Ventil AUF</button>&nbsp;";
//     html += "<button onclick=\"setRelay('OFF')\">Ventil ZU</button>";
//     html += "<h2>Motor Control</h2>";
//     html += "<input type='number' id='motorPos' placeholder='Position (Grad)' style='width:100px' />";
//     html += "<button onclick=\"setMotor(document.getElementById('motorPos').value)\">Set Motor</button><br><br>";
//     html += "<button onclick=\"setMotorIdle()\">Motor IDLE</button><br><br>";
//     html += "<h2>Motor Speed Control</h2>";
//     html += "<input type='number' id='motorSpeed' placeholder='Speed (rad/s)' style='width:100px' />";
//     html += "<button onclick=\"setMotorSpeed(document.getElementById('motorSpeed').value)\">Set Motor Speed</button>";
//     html += "</body></html>";
//     request->send(200, "text/html", html);
//   });

//   // JSON-Endpoint: Liefert ADC-, HX711-, Flow-Sensor- und ggf. weitere Werte
//   server.on("/values", HTTP_GET, [](AsyncWebServerRequest *request) {
//     int16_t adc0 = ads.readADC_SingleEnded(0);
//     int16_t adc1 = ads.readADC_SingleEnded(1);
//     float volts0 = ads.computeVolts(adc0);
//     float volts1 = ads.computeVolts(adc1);
//     // HX711 Messwerte
//     float hx711_1 = scale1.get_units(5);
//     float hx711_2 = scale2.get_units(5);
    
//     String json = "{";
//     json += "\"adc0\":" + String(adc0) + ",";
//     json += "\"volts0\":" + String(volts0, 4) + ",";
//     json += "\"adc1\":" + String(adc1) + ",";
//     json += "\"volts1\":" + String(volts1, 4) + ",";
//     json += "\"hx711_1\":" + String(hx711_1, 2) + ",";
//     json += "\"hx711_2\":" + String(hx711_2, 2) + ",";
//     json += "\"flow\":" + String(flowRate, 2);
//     json += "}";
//     request->send(200, "application/json", json);
//   });

//   // Endpoint zur Steuerung des Relais
//   server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
//     if (request->hasParam("state")) {
//       String state = request->getParam("state")->value();
//       if (state.equalsIgnoreCase("ON") || state.equals("1")) {
//         digitalWrite(relayPin, HIGH);
//         request->send(200, "text/plain", "Relay is ON");
//       }
//       else if (state.equalsIgnoreCase("OFF") || state.equals("0")) {
//         digitalWrite(relayPin, LOW);
//         request->send(200, "text/plain", "Relay is OFF");
//       }
//       else {
//         request->send(400, "text/plain", "Ungültiger Befehl");
//       }
//     }
//     else {
//       request->send(400, "text/plain", "Kein Parameter");
//     }
//   });

//   // Endpoint zur Steuerung des ODrive-Motors (Position setzen)
//   server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request) {
//     if (request->hasParam("position")) {
//       // Vor Absetzen des Positionsbefehls in den Closed-Loop-Control-Modus versetzen
//       odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
//       String posStr = request->getParam("position")->value();
//       int pos = posStr.toInt();  // Umrechnung von Grad in Bogenmaß
//       position = (pos * 6.28) / 360;  // Zielposition berechnen
//       odrive.setPosition(position);
//       request->send(200, "text/plain", "Motor position set to " + String(position));
//     } else {
//       request->send(400, "text/plain", "Kein Parameter");
//     }
//   });

//   // Endpoint zur Steuerung des Motors in den Idle-Modus
//   server.on("/motorIdle", HTTP_GET, [](AsyncWebServerRequest *request) {
//     odrive.clearErrors();
//     odrive.setState(AXIS_STATE_IDLE);
//     request->send(200, "text/plain", "Motor set to idle");
//   });

//   // Neuer Endpoint zur Steuerung der Motor-Geschwindigkeit
//   server.on("/motorspeed", HTTP_GET, [](AsyncWebServerRequest *request) {
//     if (request->hasParam("speed")) {
//       odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
//       String speedStr = request->getParam("speed")->value();
//       motorSpeed = speedStr.toFloat();
//       request->send(200, "text/plain", "Motor speed set to " + String(motorSpeed));
//     } else {
//       request->send(400, "text/plain", "Kein Parameter");
//     }
//   });

//   server.begin();
//   Serial.println("AsyncWebServer gestartet");
// }

// float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// void loop() {
//   // Update ADC-Werte einmal pro Sekunde
//   static unsigned long previousADCMillis = 0;
//   unsigned long currentMillis = millis();
//   if(currentMillis - previousADCMillis >= 100000) {
//     int16_t adc0 = ads.readADC_SingleEnded(0);
//     int16_t adc1 = ads.readADC_SingleEnded(1);
//     float volts0 = ads.computeVolts(adc0);
//     float volts1 = ads.computeVolts(adc1);
//     Serial.print(volts0);
//     Serial.print(",");
//     Serial.print(volts1);
//     float bar1 = mapFloat(volts0, 0.65, 3.3, 0, 1);
//     float bar2 = mapFloat(volts1, 0.68, 3.3, 0, 0.5);
//     Serial.print(",");
//     Serial.print(bar1);
//     Serial.print(",");
//     Serial.println(bar2);
//     previousADCMillis = currentMillis;
//   }

//   // Update HX711-Werte einmal pro Sekunde
//   static unsigned long previoushx711Millis = 0;
//   if(currentMillis - previoushx711Millis >= 100000) {
//     //hx711_1_value = scale1.get_units(3);
//     //hx711_2_value = scale2.get_units(3);
//     Serial.print("Wägezelle 1: ");
//     Serial.print(hx711_1_value);
//     Serial.print(",   Wägezelle 2: ");
//     Serial.println(hx711_2_value);
//     previoushx711Millis = currentMillis;
//   }

//   // Berechne den Flow-Wert einmal pro Sekunde
//   static unsigned long previousFlowMillis = 0;
//   if(currentMillis - previousFlowMillis >= 100000) {
//     // Beispiel: Flow (L/min) = (Anzahl Pulse pro Sekunde) / 7.5  
//     flowRate = (float)flow_count / 7.5;
//     Serial.print("Flowrate:  ");
//     Serial.println(flowRate);
//     flow_count = 0;  // Reset
//     previousFlowMillis = currentMillis;
//   }

//   // Kontinuierliche Motorsteuerung: Wenn eine Geschwindigkeit gesetzt wurde,
//   // wird die Zielposition fortlaufend aktualisiert.
//   static unsigned long previousMotorMillis = millis();
//   unsigned long currentMotorMillis = millis();
//   float dtMotor = (currentMotorMillis - previousMotorMillis) / 1000.0;
//   previousMotorMillis = currentMotorMillis;
//   // Position kontinuierlich erhöhen (oder verringern, falls motorSpeed negativ)
//   position += motorSpeed * dtMotor;
//   odrive.setPosition(position,5);
//   //odrive.setVelocity(motorSpeed,10);
//   Serial.print("1");
//   delay(10);
// }
