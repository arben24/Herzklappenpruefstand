#include <AccelStepper.h>

// ------------------------------------------------------------
// Hardware‑Pins (anpassen, falls nötig)
// ------------------------------------------------------------
#define STEP_PIN     13   // CLK‑Eingang Treiber
#define DIR_PIN      12   // DIR‑Eingang Treiber
#define ENABLE_PIN   14   // Enable (LOW = Treiber aktiv)
#define REF_PIN      15   // Endschalter (LOW, solange Schalter NICHT erreicht)

const int STEPS_PER_REV = 200;    // Schritte je Umdrehung bei aktuellem Microstepping

// ------------------------------------------------------------
// Schrittmotor‑Objekt
// ------------------------------------------------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ------------------------------------------------------------
// Betriebsarten
// ------------------------------------------------------------
enum Mode { STOP_MODE, RUN_MODE, REFERENCE_MODE };
Mode mode = STOP_MODE;

// ------------------------------------------------------------
// Globale Variablen
// ------------------------------------------------------------
float rpm = 0.0f;                 // Solldrehzahl (+/‑)

// ------------------------------------------------------------
// Serielle Kommandos parsen
// ------------------------------------------------------------
void parseSerial()
{
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();

  if (line == "STOP") {
    rpm  = 0.0f;
    stepper.setSpeed(0);
    mode = STOP_MODE;
  }
  else if (line.startsWith("RPM ")) {          // RPM <wert>
    float val = line.substring(4).toFloat();    // kann +/‑ sein → Drehrichtung
    rpm  = val;
    float stepsPerSec = (rpm * STEPS_PER_REV) / 60.0f;
    stepper.setSpeed(stepsPerSec);
    mode = RUN_MODE;
  }
  else if (line == "REF") {
    mode = REFERENCE_MODE;
  }
  else if (line == "STAT") {
    Serial.printf("MODE:%d RPM:%.2f POS:%ld\n", mode, rpm, stepper.currentPosition());
  }
  else {
    Serial.println("ERR");
  }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println(F("Konstant‑Drehzahl‑Controller – Befehle:\n"
                   "  RPM <wert>     – Drehzahl in U/min (+/‑)\n"
                   "  STOP           – Halt\n"
                   "  REF            – Referenzfahrt\n"
                   "  STAT           – Status"));

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);           // Treiber aktivieren

  pinMode(REF_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(100000);             // Obergrenze Steps/s
  // Beschleunigung wird in runSpeed() nicht verwendet, kann aber gesetzt bleiben
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop()
{
  parseSerial();

  switch (mode)
  {
    case STOP_MODE:
      // Motor hält Position (wird gehalten, da Enable LOW)
      break;

    case RUN_MODE:
      stepper.runSpeed();                 // konstante Drehzahl
      break;

    case REFERENCE_MODE:
      stepper.setSpeed(3000);             // feste Geschwindigkeit Richtung Schalter
      while (digitalRead(REF_PIN) == LOW) stepper.runSpeed();
      stepper.stop();
      stepper.setCurrentPosition(0);
      mode = STOP_MODE;
      Serial.println("REF OK");
      break;
  }
}
