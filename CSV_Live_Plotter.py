import serial
import matplotlib.pyplot as plt
import csv
import os
import json
from datetime import datetime
from time import sleep

# ------------------ BENUTZEREINGABEN UND KONFIGURATION ------------------

# Klappen-Nummer oder Name abfragen
flap_name = input("Geben Sie die Klappennummer oder den Namen ein: ").strip()

# Messzeit (in ms) abfragen und per seriellem Befehl setzen
while True:
    try:
        time_ms = int(input("Geben Sie die Messzeit in Millisekunden ein: ").strip())
        break
    except ValueError:
        print("Ungültige Eingabe. Bitte eine ganze Zahl für die Zeit in ms eingeben.")

# Anzahl der Durchläufe (Messzyklen) abfragen
while True:
    try:
        num_runs = int(input("Wie oft soll der gesamte Messprozess durchgeführt werden? ").strip())
        if num_runs < 1:
            print("Die Anzahl der Durchläufe muss mindestens 1 sein.")
            continue
        break
    except ValueError:
        print("Ungültige Eingabe. Bitte eine ganze Zahl eingeben.")

# Sensitivitäten [V/bar oder V/(l/min)] und Offsetwerte [V] – je Kanal
# (diese Werte werden in der Config-Datei mitgespeichert)
sensitivity = [47, 6.4, 6.4, 0.032]
offsets = {
    "p_diff_air": 0.41,
    "p_abs": 0.78,
    "p_diff": 0.78,
    "flow": 0.7
}

# Seriellen Port und Baudrate einstellen
PORT = '/dev/ttyUSB0'  # Beispiel für Linux; unter Windows könnte es 'COM3' oder ähnlich sein
#PORT = 'COM3'  # Beispiel für Windows
BAUDRATE = 115200

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
base_folder = "Messungen"
os.makedirs(base_folder, exist_ok=True)
folder_name = os.path.join(base_folder, f"{flap_name}_{timestamp}")
os.makedirs(folder_name, exist_ok=True)

# Pfade für Config- und CSV-Datei
config_path = os.path.join(folder_name, "config.json")
csv_path = os.path.join(folder_name, "messdaten.csv")

# Config-Dictionary zusammenstellen und in JSON-Datei schreiben
config_data = {
    "flap_name": flap_name,
    "time_ms": time_ms,
    "num_runs": num_runs,
    "sensitivity": sensitivity,
    "offsets": offsets
}

with open(config_path, 'w', encoding='utf-8') as cfg_file:
    json.dump(config_data, cfg_file, indent=4)

print(f"Konfigurationsdatei geschrieben nach: {config_path}")
print(f"Messdaten werden gespeichert in: {csv_path}")

# CSV-Datei mit Header anlegen
with open(csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'Run', 'Flow (V)', 'Flow (L/min)',
        'Differenzdruck_klein (V)', 'Differenzdruck_klein (bar)',
        'Absolutdruck (V)', 'Absolutdruck (bar)',
        'Differenzdruck (V)', 'Differenzdruck (bar)'
    ])

# ------------------------------------------------------------------------

# Serielle Schnittstelle öffnen
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

# Messzeit per seriellem Befehl an den Mikrocontroller senden
set_time_cmd = f"set time {time_ms}\n".encode('utf-8')
ser.write(set_time_cmd)
print(f"Serieller Befehl gesendet: {set_time_cmd.decode().strip()}")

# Start-Signal für Aufnahme senden
ser.write(b"motor record\n")
print("Signal 'motor record' gesendet. Warte auf Startsignal...")

# Zähler für durchgeführte Runs
run_count = 0
aufzeichnung_aktiv = False

try:
    while run_count <= num_runs:
        raw = ser.readline().decode('utf-8').strip()
        if not raw:
            continue
        print(raw)

        # Setup-Phase (zwischen "Setup start" und "Setup end")
        if raw == "Setup start":
            print(f"Run {run_count+1}/{num_runs}: Setup gestartet...")
            while True:
                raw = ser.readline().decode('utf-8').strip()
                if not raw:
                    continue
                print(raw)
                if raw == "Setup end":
                    print("Setup beendet.")
                    break

        # Start der Kennlinienaufnahme erkennen
        if raw == "Kennlinienaufnahme gestartet.":
            run_count += 1
            print(f"Run {run_count}/{num_runs}: Kennlinienaufnahme gestartet...")
            aufzeichnung_aktiv = True

        # Ende der Kennlinienaufnahme erkennen
        elif raw == "Kennlinienaufnahme beendet." and aufzeichnung_aktiv:
            print(f"Run {run_count}/{num_runs}: Kennlinienaufnahme beendet.")
            aufzeichnung_aktiv = False

            # Wenn noch weitere Runs offen sind, auf nächstes Startsignal warten
            if run_count < num_runs:
                print(f"Warte auf Startsignal für Run {run_count+1}...")
                # (Optional: Falls pro Run neuer "motor record" Befehl nötig, hier senden)
                sleep(1)
                ser.write(b"motor record\n")
                print("Signal 'motor record' gesendet. Warte auf Startsignal...")
                continue
            else:
                print("Alle Messdurchläufe abgeschlossen.")
                break

        # Datenerfassung während aktiver Aufnahme
        elif aufzeichnung_aktiv:
            teile = raw.split(',')
            if len(teile) >= 4:
                try:
                    # Rohspannungen auslesen
                    p_diff_air_v = float(teile[0])
                    p_abs_v = float(teile[1])
                    p_diff_v = float(teile[2])
                    flow_v = float(teile[3])

                    # Offset abziehen
                    p_diff_air_corr = p_diff_air_v - offsets["p_diff_air"]
                    p_abs_corr = p_abs_v - offsets["p_abs"]
                    p_diff_corr = p_diff_v - offsets["p_diff"]
                    flow_corr = flow_v - offsets["flow"]

                    # Umrechnung in physikalische Einheiten
                    p_diff_air_bar = p_diff_air_corr / sensitivity[0]
                    p_abs_bar = p_abs_corr / sensitivity[1]
                    p_diff_bar = p_diff_corr / sensitivity[2]
                    flow_calc = flow_corr / sensitivity[3]

                    # In CSV-Datei schreiben (Spalte "Run" hinzugefügt)
                    with open(csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            run_count,
                            flow_v, flow_calc,
                            p_diff_air_v, p_diff_air_bar,
                            p_abs_v, p_abs_bar,
                            p_diff_v, p_diff_bar
                        ])

                except ValueError:
                    print("Ungültige Zahlenwerte erhalten:", teile)

except KeyboardInterrupt:
    print("\nAbbruch durch Benutzer. Serielle Schnittstelle wird geschlossen.")
    ser.close()
    exit()

# ------------------------------------------------------------------------
# Nach der Aufnahme: CSV-Daten plotten (nur Bar-Werte)
flow_list = []
abs_bar_list = []
diff_bar_list = []
diff_air_bar_list = []

with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            flow_list.append(float(row['Flow (L/min)']))
            abs_bar_list.append(float(row['Absolutdruck (bar)']))
            diff_bar_list.append(float(row['Differenzdruck (bar)']))
            diff_air_bar_list.append(float(row['Differenzdruck_klein (bar)']))
        except ValueError:
            continue

if flow_list:
    plt.figure()
    plt.plot(flow_list, abs_bar_list, label="Absolutdruck (bar)")
    plt.plot(flow_list, diff_bar_list, label="Differenzdruck (bar)")
    plt.plot(flow_list, diff_air_bar_list, label="Differenzdruck klein (bar)")
    plt.xlabel("Flow (L/min)")
    plt.ylabel("Druck (bar)")
    plt.title(f"Kennlinienaufnahme für '{flap_name}' ({num_runs} Runs)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("Keine Daten zum Plotten gefunden.")

ser.close()
