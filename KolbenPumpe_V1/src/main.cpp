#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definition der Pins
#define STEP_PIN      13
#define DIR_PIN       12
#define ENABLE_PIN    14
#define BUTTON_L_PIN  27     // Button zum Moduswechsel (mit internem Pullup)
#define BUTTON_R_PIN  26     // Button für die Referenzierung
#define REF_PIN       15
#define POT_PIN       33     // Potentiometer für Oszillationsamplitude

// OLED Display Parameter
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS  0x3C  // Standard I2C Adresse

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Motor-Konstanten
const int stepsPerRevolution = 1600;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

//spindelkonstanten
const int anstieg = 5; //5mm Anstieg der spinedl das heiß 5mm pro umdrehung

// Definition der Modi
enum Mode {
  STOP_MODE,
  POSITION_MODE,
  OSCILLATION_MODE,
  REFERENCE_MODE   // Neuer Modus zur Referenzierung
};

Mode currentMode = STOP_MODE;

const float trajectory_mm[] = {
  0, 10, 20, 40, 30, 20, 10, 5, 0
};
const int traj_length = sizeof(trajectory_mm) / sizeof(trajectory_mm[0]); // Anzahl der Punkte in der Trajektorie


// Variablen für den Positionmodus
long targetPosition = 0;

// Variablen für den Oszillationsmodus
int puls = 0;
bool oscillationDirection = true; // true: vorwärts, false: rückwärts
int trajectoryIndex = 0;

// Button Entprellung
unsigned long lastButtonPress = 0;
unsigned long lastReferenceButtonPress = 0;
const unsigned long debounceDelay = 200;

void moveStepperLinear(long targetPos, unsigned long durationMs) {
  long startPos = stepper.currentPosition();
  long distance = targetPos - startPos;
  float durationSec = durationMs / 1000.0;
  // Berechne die erforderliche Geschwindigkeit in Schritten pro Sekunde
  float requiredSpeed = distance / durationSec;
  
  stepper.setSpeed(requiredSpeed);
  stepper.moveTo(targetPos);
  
  unsigned long startTime = millis();
  
  // Führe die Bewegung für die vorgegebene Dauer aus
  while (millis() - startTime < durationMs) {
    stepper.runSpeed();
  }
  
  // Gewährleiste, dass der Zielpunkt exakt erreicht wird
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void update_display() {
  // Lösche den Display-Puffer
  display.clearDisplay();
  
  // Setze Textgröße und -farbe
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Setze den Cursor an den Anfang (0,0)
  display.setCursor(0, 0);
  
  // Zeige den aktuellen Modus an
  display.print("Modus: ");
  switch(currentMode) {
    case STOP_MODE:
      display.print("Stopp");
      break;
    case POSITION_MODE:
      display.print("Position");
      break;
    case OSCILLATION_MODE:
      display.print("Oszillation");
      break;
    case REFERENCE_MODE:
      display.print("Referenzierung");
      break;
  }
  
  display.setCursor(0, 10);
  // Zeige die aktuelle Motorposition an
  display.print("Pos: ");
  display.print(-(stepper.currentPosition()*anstieg)/stepsPerRevolution);
  display.print(" mm");

  display.setCursor(0, 20);
  // Zeige die Zielposition an
  display.print("Ziel: ");
  display.print(targetPosition);
  display.print(" mm");
  
  // Falls im Oszillationsmodus, auch die Amplitude anzeigen
  if(currentMode == OSCILLATION_MODE) {
    display.setCursor(0, 30);
    display.print("PULS(Schlag/Min): ");
    display.println(puls);
  }
  
  // Zeige zusätzlichen statischen Text an
  display.setCursor(50, 50);
  display.println("SoKoRoMed");
  
  // Aktualisiere das Display
  display.display();
}

void setup() {
  Serial.begin(115200);

  // ESP32-spezifische I²C-Initialisierung: SDA an GPIO 21, SCL an GPIO 22
  Wire.begin(21, 22);

  // Motor Enable-Pin konfigurieren (aktiv LOW)
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  // Buttons mit internem Pullup initialisieren
  pinMode(BUTTON_L_PIN, INPUT_PULLUP);
  pinMode(BUTTON_R_PIN, INPUT_PULLUP);
  pinMode(REF_PIN, INPUT_PULLUP);
  
  // Konfiguration des Motors: Geschwindigkeit und Beschleunigung
  stepper.setMaxSpeed(80000);
  stepper.setAcceleration(120000);
  
  // OLED Display initialisieren
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED Display konnte nicht initialisiert werden.");
    while (true);
  }
  display.clearDisplay();
  display.display();

  Serial.println("Gib im Positionmodus die Umdrehungen (als Float) per serieller Eingabe ein.");
}

void loop() {
  // Button-Check zum Wechseln des Modus (BUTTON_L_PIN, active LOW)
  if (digitalRead(BUTTON_L_PIN) == LOW && (millis() - lastButtonPress) > debounceDelay) {
    // Moduszyklus: STOP -> POSITION -> OSZILLATION -> REFERENZ -> STOP ...
    if (currentMode == REFERENCE_MODE) {
      currentMode = STOP_MODE;
    } else {
      currentMode = (Mode)(currentMode + 1);
    }
    lastButtonPress = millis();
  }

  switch (currentMode) {
    case STOP_MODE:
      // Im Stopp-Modus: Motor anhalten
      stepper.stop();
      break;
      
    case POSITION_MODE: {


      int potVal = analogRead(POT_PIN);
      // Mapping des Potentiometerwertes (0-1023) auf eine Amplitude (z.B. 0 bis 3000 Schritte)
      targetPosition = map(potVal, 0, 4095, 0, 100);     //amplitude in mm also 10 cm maximal zulässig
      Serial.println(targetPosition);


      if (digitalRead(BUTTON_R_PIN) == LOW && (millis() - lastReferenceButtonPress) > debounceDelay) {

        stepper.moveTo(-(targetPosition*stepsPerRevolution)/anstieg); 

        lastReferenceButtonPress = millis();

      }  


      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      break;
        }

        case OSCILLATION_MODE: {
        
          
          // BPM (Puls) über Poti updaten
          int potValue = analogRead(POT_PIN);
          puls = map(potValue, 0, 4095, 0, 120);
          // Ein Herzschlag entspricht 60000ms geteilt durch die BPM
          int heartbeatPeriod = (puls > 0) ? (60000 / puls) : 0;
          // Jeder Trajektorienpunkt bekommt einen Anteil des gesamten Herzschlags
          int segmentDuration = (traj_length > 0 && heartbeatPeriod > 0) ? (heartbeatPeriod / traj_length) : 0;
          
          // Neuen Zielpunkt aus der Trajektorie berechnen
          long currentTarget = -(trajectory_mm[trajectoryIndex] * stepsPerRevolution) / anstieg;
          
          // Bewege den Schrittmotor linear zum Zielpunkt über die Segmentdauer
          moveStepperLinear(currentTarget, segmentDuration);
          
          trajectoryIndex = trajectoryIndex + 1 ;
          if(trajectoryIndex >= traj_length) {
            trajectoryIndex = 0; // Zurück zum ersten Punkt der Trajektorie
          }
          
          break;
        }

        case REFERENCE_MODE: {
      // Im Referenzierungsmodus: Durch Betätigung von BUTTON_R_PIN wird die Referenzierung gestartet.
      if (digitalRead(BUTTON_R_PIN) == LOW && (millis() - lastReferenceButtonPress) > debounceDelay) {
        Serial.println("Starte Referenzierung...");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Modus: Referenzierung");
        display.setCursor(0, 20);
        display.print("Starte Referenzierung");
        // Beispielhafte Referenzierung: Motor fährt eine feste Strecke (hier -1000000 Schritte)
        stepper.moveTo(1000000);
        while (digitalRead(REF_PIN) == LOW) {
          stepper.run();
          delay(2);
        }
        // Setze die aktuelle Position als Nullpunkt
        stepper.setCurrentPosition(0);
        Serial.println("Referenzierung abgeschlossen.");
        display.print("Referenzierung abgeschlossen");
        lastReferenceButtonPress = millis();
        // Optional: Nach der Referenzierung in den STOP_MODE wechseln
        currentMode = STOP_MODE;
      }
      break;
    }
  }
  
  update_display();
}
