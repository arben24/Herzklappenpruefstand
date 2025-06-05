#include <Adafruit_ADS1X15.h>
#include <ODriveUART.h>

// ADS1015 (12-Bit)
Adafruit_ADS1015 ads;

// ODrive Motor
HardwareSerial& odrive_serial = Serial2;
ODriveUART odrive(odrive_serial);
int baudrate = 115200;
float position = 0;   // Aktuelle Zielposition (in Bogenmaß)
float motorSpeed = 0; // Motor-Geschwindigkeit (rad/s)
bool isSpeedMode = false; // Flag für den Geschwindigkeitsmodus
bool isRecording = false; // Flag für den Kennlinienaufnahmemodus

// Offset in Volt bei 0 kPa (z.B. gemessen, wenn der Sensor offen ist)
const float VOLTAGE_OFFSET = 0.18f;

// Sensitivität pro Kanal (in Volt pro kPa)
//const float SENSOR_SENSITIVITY[4] = {0.45f, 0.50f, 0.40f, 0.55f}; // Beispielwerte für 4 Kanäle
const float predefinedVelocity[8] = {-1.0f,-2.0f, -3.0f, -5.0f, -6.0f, -8.0f, -10.0f, -12.0f}; // Beispielwerte für die Geschwindigkeit
int time_for_sensor_recording = 20; // Zeit in ms 


unsigned long prevMotorMillis = 0;

void updateMotor(bool print = false) {
  unsigned long currentMillis = millis();
  if (currentMillis - prevMotorMillis <= 20) {
    // Motorsteuerung nur im Geschwindigkeitsmodus
    return;
  }
  if (!isSpeedMode) {
    prevMotorMillis = currentMillis;
    return;
  }

  float dtMotor = (currentMillis - prevMotorMillis) / 1000.0; // Zeitdifferenz in Sekunden
  prevMotorMillis = currentMillis;

  position += motorSpeed * dtMotor;
  odrive.setPosition(position);

if (print) {
    Serial.print("Motorposition: ");
    Serial.print(position, 4);
    Serial.print(" rad, Motor-Geschwindigkeit: ");
    Serial.println(motorSpeed, 4);
  }
}

// Funktion zum Einlesen und Mitteln der Sensorwerte
void readAndAverageSensorValues(int numSamples, float averagedVoltages[4],bool print = false) {
  float voltageSums[4] = {0, 0, 0, 0};

  // Erst nacheinander numSamples auslesen und dabei alle Kanäle abfragen
  for (int i = 0; i < numSamples; i++) {
    for (int channel = 0; channel < 4; channel++) {
      int16_t adcValue = ads.readADC_SingleEnded(channel);
      float voltage = ads.computeVolts(adcValue);
      voltageSums[channel] += voltage;
    }
    if (isSpeedMode)
    {
      updateMotor();
    }
    
  }
   // Mittelwert berechnen und ausgeben für jeden Kanal
  for (int channel = 0; channel < 4; channel++) {
    averagedVoltages[channel] = voltageSums[channel] / numSamples;
if (print)
{
      Serial.print("Kanal ");
      Serial.print(channel);
      Serial.print(": ");
      Serial.print(averagedVoltages[channel], 4);
      Serial.print(" V");
      Serial.println(" ");

  
  }
}
}


void readAndAverageSensorValuesOverTime(int time_ms, float averagedVoltages[4],bool print = false) {
  float voltageSums[4] = {0, 0, 0, 0};

  int numSamples = 0;
  unsigned long startMillis = millis();
  while (millis() - startMillis < time_ms){
    for (int channel = 0; channel < 4; channel++) {
        int16_t adcValue = ads.readADC_SingleEnded(channel);
        float voltage = ads.computeVolts(adcValue);
        voltageSums[channel] += voltage;
      }
    if (isSpeedMode){
      updateMotor();
    }
    numSamples++;
  }

  for (int channel = 0; channel < 4; channel++) {
    averagedVoltages[channel] = voltageSums[channel] / numSamples;
    if (print){
      Serial.print("Kanal ");
      Serial.print(channel);
      Serial.print(": ");
      Serial.print(averagedVoltages[channel], 4);
      Serial.print(" V");
      Serial.println(" ");
    }
  }
}

// Funktion für die Kennlinienaufnahme
void recordCurve() {
  static unsigned long prevRecordMillis = 0;
  unsigned long currentMillis = millis();

  isSpeedMode = true; // Wechsel in den Geschwindigkeitsmodus

  motorSpeed = 0; 
  ODriveFeedback feedback = odrive.getFeedback();
  position = feedback.pos; // Aktualisiere die Position im Programm


  for (int i = 0; i < sizeof(predefinedVelocity) / sizeof(predefinedVelocity[0]); i++) {
    motorSpeed = predefinedVelocity[i];
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {
      updateMotor();
    }

    unsigned long measureTime = millis();
    float averagedVoltages[4];

    readAndAverageSensorValuesOverTime(time_for_sensor_recording, averagedVoltages,false);

    unsigned long stableTime = millis();
    while (millis() - stableTime < 3000) {
      updateMotor();
    }

    for (int channel = 0; channel < 4; channel++) {
      Serial.print(averagedVoltages[channel], 4);
      Serial.print(",");
    }
    Serial.println();
  }

  Serial.println("Kennlinienaufnahme beendet.");
  isSpeedMode = false; // Wechsel zurück in den Idle-Modus
  isRecording = false; // Beende die Kennlinienaufnahme
  odrive.setState(AXIS_STATE_IDLE); // Setze den Motor in den Idle-Modus

}


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Setup start");

  // ADS1015 initialisieren
  if (!ads.begin()) {
    Serial.println("Fehler: ADS1015 nicht gefunden!");
    while (1) { delay(100); }
  }

  // ODrive initialisieren
  odrive_serial.begin(baudrate, SERIAL_8N1, 16, 17);
  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  Serial.println("Found ODrive");

  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  Serial.println("ODrive running!");
  // Starte im Idle-Modus und lese die aktuelle Position
  odrive.setState(AXIS_STATE_IDLE);
  position = odrive.getPosition();

  // Gib verfügbare Befehle aus
  Serial.println("Verfügbare Befehle:");
  Serial.println("  motor pos X      -- fährt den Motor auf Position X (Grad)");
  Serial.println("  motor speed X    -- setzt die Motor-Geschwindigkeit (rad/s)");
  Serial.println("  motor record     -- startet die Kennlinienaufnahme");
  Serial.println("  motor idle       -- versetzt den Motor in den Idle-Modus");

  Serial.println("Setup end");
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("motor pos ")) {
      // Beispiel: "motor pos 90" (Grad)
      String posStr = cmd.substring(10);  // Nach "motor pos "
      int posDeg = posStr.toInt();
      position = (posDeg * 6.28) / 360;     // Grad in Bogenmaß
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      odrive.setPosition(position);
      isSpeedMode = false; // Wechsel in den Positionsmodus
      isRecording = false; // Beende die Kennlinienaufnahme
      Serial.print("Motorposition auf ");
      Serial.print(position, 4);
      Serial.println(" rad gesetzt");
    }
    if (cmd.startsWith("set time ")) {
      // Beispiel: "motor pos 90" (Grad)
      String timeStr = cmd.substring(9);  // Nach "motor pos "
      int timeint = timeStr.toInt();
      time_for_sensor_recording = timeint ;     // Grad in Bogenmaß
      Serial.print("Sensor Time auf ");
      Serial.print(time_for_sensor_recording);
      Serial.println(" ms gesetzt");
    }
    else if (cmd.startsWith("motor speed ")) {
      // Beispiel: "motor speed 0.5" (rad/s)
      String speedStr = cmd.substring(12); // Nach "motor speed "
      motorSpeed = speedStr.toFloat();
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      isSpeedMode = true; // Wechsel in den Geschwindigkeitsmodus
      isRecording = false; // Beende die Kennlinienaufnahme

      // Synchronisiere die aktuelle Position des Motors
      ODriveFeedback feedback = odrive.getFeedback();
      position = feedback.pos; // Aktualisiere die Position im Programm
      Serial.print("Motor-Geschwindigkeit auf ");
      Serial.print(motorSpeed, 4);
      Serial.println(" rad/s gesetzt");
      Serial.print("Aktuelle Motorposition synchronisiert: ");
      Serial.println(position, 4);
    }
    else if (cmd.equalsIgnoreCase("motor record")) {
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      isRecording = true; // Aktiviere die Kennlinienaufnahme
      isSpeedMode = false; // Beende den Geschwindigkeitsmodus
      Serial.println("Kennlinienaufnahme gestartet.");
    }
    else if (cmd.equalsIgnoreCase("motor idle")) {
      odrive.clearErrors();
      odrive.setState(AXIS_STATE_IDLE);
      isSpeedMode = false; // Kein Modus aktiv
      isRecording = false; // Beende die Kennlinienaufnahme
      Serial.println("Motor in Idle-Modus");
    }
    else {
      Serial.print("Unbekannter Befehl: ");
      Serial.println(cmd);
      Serial.println("Befehle: motor pos X, motor speed X, motor record, motor idle");
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Sensorwerte (ADC) alle 10 ms auslesen und ausgeben
  static unsigned long prevADCMillis = 0;
  if (currentMillis - prevADCMillis >= 1000) {
    float averagedVoltages[4];
    //readAndAverageSensorValues(10, averagedVoltages,true); // 10 Werte mitteln
    prevADCMillis = currentMillis;
  }



  // Aktualisiere die Motorsteuerung nur im Geschwindigkeitsmodus
  updateMotor();

  // Führe die Kennlinienaufnahme aus, falls aktiv
  if (isRecording) {
    recordCurve();
  }

  // Verarbeite serielle Befehle
  processSerialCommands();
  // ODriveFeedback feedback = odrive.getFeedback();
  // position = feedback.pos; // Aktualisiere die Position im Programm
  // Serial.print("Aktuelle Motorposition: ");
  // Serial.println(position, 4);

  delay(1);
}