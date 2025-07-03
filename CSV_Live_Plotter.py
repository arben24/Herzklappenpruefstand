import serial
import matplotlib.pyplot as plt
import csv
import os
import json
from datetime import datetime
from time import sleep
import collections
import numpy as np
import dearpygui.dearpygui as dpg

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


kalibrieren = input("Soll eine Offset-Kalibrierung durchgeführt werden? (j/n): ").strip().lower() == "j"



# Sensitivitäten [V/bar oder V/(l/min)] und Offsetwerte [V] – je Kanal
# (diese Werte werden in der Config-Datei mitgespeichert)
sensitivity = [47, 6.4, 6.4, 0.032]

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


print(f"Konfigurationsdatei geschrieben nach: {config_path}")
print(f"Messdaten werden gespeichert in: {csv_path}")

# CSV-Datei mit Header anlegen
with open(csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'Run',
        'Flow (V)', 
        'Flow_center (L/min)', 'Flow_lower (L/min)', 'Flow_upper (L/min)',
        'Differenzdruck_klein (V)',
        'p_diff_air_center (bar)', 'p_diff_air_lower (bar)', 'p_diff_air_upper (bar)',
        'Absolutdruck (V)', 'Absolutdruck (bar)',
        'Differenzdruck (V)', 
        'Differenzdruck_center (bar)', 'Differenzdruck_lower (bar)', 'Differenzdruck_upper (bar)'
        ])



# ---------- MPX5010-Spezifikation (0 … 85 °C) ----------
VS              = 5.0        # Versorgungsspannung in V
ERROR_KPA       = 0.5        # ±-Fehler laut Datenblatt 
SLOPE_KPA_PER_V = 1 / (0.09 * VS)   # = 2.2222 kPa / V 

def p5010_kpa(v_out_corr):
    """Zentralwert (kPa) aus offset-korrigiertem Sensor-Vout."""
    return SLOPE_KPA_PER_V * v_out_corr

def p5010_band_kpa(v_out_corr):
    """(lower, central, upper) jeweils in kPa."""
    p = p5010_kpa(v_out_corr)
    return p - ERROR_KPA, p, p + ERROR_KPA

# ---------- SHD 692-I (0–0.5 bar, 4–20 mA → 0.8–4.0 V) ----------
SLOPE_BAR_PER_V = 0.15625      # 0.5 bar / 3.2 V
OFFSET_VOLT     = 0.8          # Spannung bei 4 mA
ERR_SENSOR_BAR  = 0.0065       # ±1.3 % FS


def shd692_press_bar_from_v(v_out):
    """Zentralwert (bar) aus gemessener Spannung in V."""
    return SLOPE_BAR_PER_V * (v_out)

def shd692_band_bar_from_v(v_out, err_bar=ERR_SENSOR_BAR):
    """(lower, center, upper) in bar für gegebenes U_out."""
    p = shd692_press_bar_from_v(v_out)
    return p - err_bar, p, p + err_bar

# ---------- Keyence FD-H20, 4–20 mA via 200 Ω ----------
FS_LPM          = 100.0        # 60.0 für 15 A-Variante
SLOPE_LPM_PER_V = FS_LPM / 3.2 # 31.25 oder 18.75
OFFSET_VOLT     = 0.8
ERR_LOW_LPM     = 0.003 * FS_LPM   # 0,30 bzw. 0,18

def flow_central(v_out):
    """Momentanwert in L/min (Zentralwert)."""
    return SLOPE_LPM_PER_V * (v_out)

def flow_band(v_out):
    """Unter-, Zentral-, Obergrenze in L/min (Worst Case)."""
    q = flow_central(v_out)
    if q >= 0.1 * FS_LPM:
        err = 0.03 * q                # ±3 % of reading
    else:
        err = ERR_LOW_LPM            # ±0,3 % FS

    return q - err, q, q + err


# ------------------------------------------------------------------------

# Serielle Schnittstelle öffnen
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

if kalibrieren:
    print("Starte Kalibrierung...")
    ser.write(b"starte kalibrierung\n")
    offsets = {}
    for _ in range(4):
        line = ser.readline().decode('utf-8').strip()
        # Erwartetes Format: "Kanal X: Y.YYYY V"
        if line.startswith("Kanal"):
            try:
                parts = line.split(":")
                kanal = int(parts[0].split()[1])
                wert = float(parts[1].replace("V", "").strip())
                # Mapping wie im offsets-Original
                if kanal == 0:
                    offsets["p_diff_air"] = wert
                elif kanal == 1:
                    offsets["p_abs"] = wert
                elif kanal == 2:
                    offsets["p_diff"] = wert
                elif kanal == 3:
                    offsets["flow"] = wert
                print(f"Offset Kanal {kanal}: {wert} V")
            except Exception as e:
                print("Fehler beim Einlesen der Kalibrierung:", line, e)
    print("Offset-Kalibrierung abgeschlossen.")
else:
    # Standard-Offsets
    offsets = {
        "p_diff_air": 0.2,
        "p_abs": 0.8,
        "p_diff": 0.8,
        "flow": 0.8
    }


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
                    #p_diff_air_bar = p5010_kpa(p_diff_air_corr) / 100.0  # kPa in bar
                    p_diff_air_lower, p_diff_air_center, p_diff_air_upper = p5010_band_kpa(p_diff_air_corr)
                    p_diff_air_lower /= 100.0  # kPa in bar
                    p_diff_air_upper /= 100.0  # kPa in bar
                    p_diff_air_center /= 100.0  # kPa in bar
                    p_abs_bar = p_abs_corr / sensitivity[1]
                    #p_diff_bar = p_diff_corr / sensitivity[2]
                    p_diff_lower, p_diff_center, p_diff_upper = shd692_band_bar_from_v(p_diff_corr)
                    #flow_calc = flow_corr / sensitivity[3]
                    flow_lower, flow_center, flow_upper = flow_band(flow_corr)

                    # In CSV-Datei schreiben (Spalte "Run" hinzugefügt)
                    with open(csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            run_count,
                            flow_v, 
                            flow_center, flow_lower, flow_upper,
                            p_diff_air_v,
                            round(p_diff_air_center, 3), round(p_diff_air_lower, 3), round(p_diff_air_upper, 3),
                            p_abs_v,  p_abs_bar,
                            p_diff_v, 
                            round(p_diff_center, 3), round(p_diff_lower, 3), round(p_diff_upper, 3)
                        ])


                except ValueError:
                    print("Ungültige Zahlenwerte erhalten:", teile)

except KeyboardInterrupt:
    print("\nAbbruch durch Benutzer. Serielle Schnittstelle wird geschlossen.")
    ser.close()
    exit()

# ------------------ PLOTTING DER MESSDATEN ------------------
#every run is a separate line in the plot
# runs = collections.defaultdict(lambda: {"flow": [], "diff_bar_center": [], "diff_bar_lower": [],"diff_bar_upper": [], "diff_air_bar": [], "diff_air_lower": [], "diff_air_upper": []})

# with open(csv_path, 'r') as f:
#     reader = csv.DictReader(f)
#     for row in reader:
#         try:
#             run_idx = int(row['Run'])
#             runs[run_idx]["flow"].append(float(row['Flow (L/min)']))
#             runs[run_idx]["diff_bar_center"].append(float(row['Differenzdruck_center (bar)']))
#             runs[run_idx]["diff_bar_lower"].append(float(row['Differenzdruck_lower (bar)']))
#             runs[run_idx]["diff_bar_upper"].append(float(row['Differenzdruck_upper (bar)']))
#             runs[run_idx]["diff_air_bar"].append(float(row['p_diff_air_center (bar)']))
#             runs[run_idx]["diff_air_lower"].append(float(row['p_diff_air_lower (bar)']))
#             runs[run_idx]["diff_air_upper"].append(float(row['p_diff_air_upper (bar)']))
#         except ValueError:
#             continue

# if runs:
#     plt.figure()
#     colors = plt.cm.get_cmap('tab10', len(runs))
#     for idx, (run_idx, data) in enumerate(sorted(runs.items())):
#         plt.plot(data["flow"], data["diff_bar_center"], label=f"Differenzdruck.center Run {run_idx}", color=colors(idx), linestyle='-')
#         plt.plot(data["flow"], data["diff_bar_lower"], label=f"Differenzdruck.lower Run {run_idx}", color=colors(idx), linestyle='-.')
#         plt.plot(data["flow"], data["diff_bar_upper"], label=f"Differenzdruck.upper Run {run_idx}", color=colors(idx), linestyle='-.')
#         # Plot für den Differenzdruck der Luft
#         plt.plot(data["flow"], data["diff_air_bar"], label=f"Diff.klein Run {run_idx}", color=colors(idx), linestyle=':')
#         plt.plot(data["flow"], data["diff_air_upper"], label=f"Diff.klein upper Run {run_idx}", color=colors(idx), linestyle='-.')
#         plt.plot(data["flow"], data["diff_air_lower"], label=f"Diff.klein lower Run {run_idx}", color=colors(idx), linestyle='-.')
#         plt.fill_between(data["flow"], data["diff_bar_lower"], data["diff_bar_upper"], color=colors(idx), alpha=0.1)
#         plt.fill_between(data["flow"], data["diff_air_lower"], data["diff_air_upper"], color=colors(idx), alpha=0.1)
#     plt.xlabel("Flow (L/min)")
#     plt.ylabel("Druck (bar)")
#     plt.title(f"Kennlinienaufnahme für '{flap_name}' ({num_runs} Runs)")
#     plt.grid(True)
#     plt.legend(fontsize='small', ncol=2)
#     plt.tight_layout()
#     plt.show()
# else:
#     print("Keine Daten zum Plotten gefunden.")


runs = collections.defaultdict(
    lambda: {"flow_center": [], "flow_lower": [], "flow_upper": [],
             "diff_bar_center": [], "diff_bar_lower": [], "diff_bar_upper": [],
             "diff_air_center": [], "diff_air_lower": [], "diff_air_upper": []}
)

with open(csv_path, 'r') as f:
    rdr = csv.DictReader(f)
    for row in rdr:
        try:
            i = int(row['Run'])
            # Flow
            runs[i]["flow_center"].append(float(row['Flow_center (L/min)']))
            runs[i]["flow_lower"].append(float(row['Flow_lower (L/min)']))
            runs[i]["flow_upper"].append(float(row['Flow_upper (L/min)']))
            # diff_bar
            runs[i]["diff_bar_center"].append(float(row['Differenzdruck_center (bar)']))
            runs[i]["diff_bar_lower"].append(float(row['Differenzdruck_lower (bar)']))
            runs[i]["diff_bar_upper"].append(float(row['Differenzdruck_upper (bar)']))
            # diff_air
            runs[i]["diff_air_center"].append(float(row['p_diff_air_center (bar)']))
            runs[i]["diff_air_lower"].append(float(row['p_diff_air_lower (bar)']))
            runs[i]["diff_air_upper"].append(float(row['p_diff_air_upper (bar)']))
        except ValueError:
            continue






plt.figure()
cmap = plt.cm.get_cmap('tab10', len(runs))

for idx, (run_idx, d) in enumerate(sorted(runs.items())):
    color = cmap(idx)

    # --- Differenzdruck groß ---
    x  = d["flow_center"]
    xe = [np.array(x) - np.array(d["flow_lower"]),
          np.array(d["flow_upper"]) - np.array(x)]
    y  = d["diff_bar_center"]
    ye = [np.array(y) - np.array(d["diff_bar_lower"]),
          np.array(d["diff_bar_upper"]) - np.array(y)]

    plt.errorbar(x, y, xerr=xe, yerr=ye,
                 fmt='o', ms=3, lw=1, capsize=2,
                 color=color,
                 label=f"Differenzdruck Run {run_idx}")

    # --- Differenzdruck klein (optional) ---
    y2  = d["diff_air_center"]
    ye2 = [np.array(y2) - np.array(d["diff_air_lower"]),
           np.array(d["diff_air_upper"]) - np.array(y2)]

    plt.errorbar(x, y2, xerr=xe, yerr=ye2,
                 fmt='s', ms=3, lw=1, capsize=2,
                 color=color, alpha=0.6,
                 label=f"Diff.klein Run {run_idx}")


plt.xlabel("Flow (L/min)")
plt.ylabel("Druck (bar)")
plt.title(f"Kennlinienaufnahme '{flap_name}' – {num_runs} Runs")
plt.grid(alpha=0.3)
plt.legend(fontsize='x-small', ncol=2)
plt.tight_layout()
plt.show()



ser.close()
