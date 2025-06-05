import serial
import matplotlib.pyplot as plt
import csv
import os

# --------- EINSTELLUNGEN ---------
PORT = 'COM6'
BAUDRATE = 115200
CSV_DATEI = 'messdaten.csv'

# Sensitivität [V/bar oder V/(l/min)]
sensitivity = [47, 6.4, 6.4, 0.032]

# Offsetwerte [V] – je Kanal
offsets = {
    "p_diff_air": 0.41,
    "p_abs": 0.78,
    "p_diff": 0.78,
    "flow": 0.7
}
# ---------------------------------

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

with open(CSV_DATEI, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'Flow (V)', 'Flow (L/min)',
        'Differenzdruck_klein (V)', 'Differenzdruck_klein (bar)',
        'Absolutdruck (V)', 'Absolutdruck (bar)',
        'Differenzdruck (V)', 'Differenzdruck (bar)'
    ])

aufzeichnung_aktiv = False

print("Warte auf Startsignal...")

ser.write(b'motor record\n')

while True:
    try:
        raw = ser.readline().decode('utf-8').strip()
        if not raw:
            continue
        print(raw)

        if raw == "Setup start":
            print("Setup gestartet...")
            while raw != "Setup end":
                raw = ser.readline().decode('utf-8').strip()
                if not raw:
                    continue
                print(raw)

        if raw == "Kennlinienaufnahme gestartet.":
            print("Kennlinienaufnahme gestartet...")
            aufzeichnung_aktiv = True

        elif raw == "Kennlinienaufnahme beendet.":
            print("Kennlinienaufnahme beendet.")
            break

        elif aufzeichnung_aktiv:
            teile = raw.split(',')
            if len(teile) >= 4:
                try:
                    # Rohspannungen
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

                    # In CSV-Datei schreiben
                    with open(CSV_DATEI, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            flow_v, flow_calc,
                            p_diff_air_v, p_diff_air_bar,
                            p_abs_v, p_abs_bar,
                            p_diff_v, p_diff_bar
                        ])

                except ValueError:
                    print("Ungültige Zahlenwerte:", teile)

    except KeyboardInterrupt:
        print("Abbruch durch Benutzer.")
        ser.close()
        exit()

# Nach der Aufnahme: CSV-Daten plotten (nur Bar-Werte)
flow_list = []
abs_bar_list = []
diff_bar_list = []
diff_air_bar_list = []

with open(CSV_DATEI, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            flow_list.append(float(row['Flow (L/min)']))
            abs_bar_list.append(float(row['Absolutdruck (bar)']))
            diff_bar_list.append(float(row['Differenzdruck (bar)']))
            diff_air_bar_list.append(float(row['Differenzdruck_klein (bar)']))
        except ValueError:
            continue

plt.figure()
plt.plot(flow_list, abs_bar_list, label="Absolutdruck (bar)", color="blue")
plt.plot(flow_list, diff_bar_list, label="Differenzdruck (bar)", color="red")
plt.plot(flow_list, diff_air_bar_list, label="Differenzdruck klein (bar)", color="green")
plt.xlabel("Flow (L/min)")
plt.ylabel("Druck (bar)")
plt.title("Kennlinienaufnahme (in bar)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


ser.close()
