import os
import csv
import json
import serial
import collections
import numpy as np
import dearpygui.dearpygui as dpg
from datetime import datetime

# ---------- Sensor-Conversion-Funktionen ----------
VS              = 5.0
ERROR_KPA       = 0.5
SLOPE_KPA_PER_V = 1/(0.09*VS)

def p5010_band_kpa(v_out_corr):
    p = SLOPE_KPA_PER_V * v_out_corr
    return (p-ERROR_KPA, p, p+ERROR_KPA)

SLOPE_BAR_PER_V = 0.15625
ERR_SENSOR_BAR  = 0.0065

def shd692_band_bar_from_v(v_out, err_bar=ERR_SENSOR_BAR):
    p = SLOPE_BAR_PER_V * v_out
    return (p-err_bar, p, p+err_bar)

FS_LPM       = 100.0
ERR_LOW_LPM  = 0.003 * FS_LPM
SLOPE_LPM_PER_V = FS_LPM/3.2

def flow_band(v_out):
    q = SLOPE_LPM_PER_V * v_out
    err = 0.03*q if q >= 0.1*FS_LPM else ERR_LOW_LPM
    return (q-err, q, q+err)

# ---------- blockierende Mess-Routine ----------
def run_measurement_blocking():
    # 1) GUI-Eingaben
    flap      = dpg.get_value("inp_flap")
    t_ms      = dpg.get_value("inp_time")
    n_runs    = dpg.get_value("inp_runs")
    do_cal    = dpg.get_value("chk_kal")
    port      = dpg.get_value("inp_port")
    baud      = dpg.get_value("inp_baud")
    offsets   = {}

    # 2) Mess-Ordner & Config
    ts     = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = os.path.join("Messungen", f"{flap}_{ts}")
    os.makedirs(folder, exist_ok=True)
    cfg = {"flap_name": flap, "time_ms": t_ms, "num_runs": n_runs,
           "kalib": do_cal, "port": port, "baud": baud}
    with open(os.path.join(folder,"config.json"),"w") as f:
        json.dump(cfg, f, indent=4)

    # 3) CSV-Header
    csv_path = os.path.join(folder,"messdaten.csv")
    with open(csv_path,"w",newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            'Run',
            'Flow (V)',
            'Flow_center (L/min)','Flow_lower (L/min)','Flow_upper (L/min)',
            'Differenzdruck_klein (V)',
            'p_diff_air_center (bar)','p_diff_air_lower (bar)','p_diff_air_upper (bar)',
            'Absolutdruck (V)','Absolutdruck (bar)',
            'Differenzdruck (V)',
            'Differenzdruck_center (bar)','Differenzdruck_lower (bar)','Differenzdruck_upper (bar)'
        ])

    # 4) Serielle Schnittstelle öffnen
    ser = serial.Serial(port, baud, timeout=1)

    # 5) Kalibrierung falls gewünscht
    if do_cal:
        ser.write(b"starte kalibrierung\n")
        for _ in range(4):
            line = ser.readline().decode().strip()
            # hier könntest Du offsets extrahieren wie vorher
        # Beispiel-Defaults, falls keine automatische Kalibrierung implementiert:
        offsets = {"p_diff_air":0.2, "p_abs":0.8, "p_diff":0.8, "flow":0.8}
    else:
        offsets = {"p_diff_air":0.2, "p_abs":0.8, "p_diff":0.8, "flow":0.8}

    # 6) Mikrocontroller konfigurieren
    ser.write(f"set time {t_ms}\n".encode())
    ser.write(b"motor record\n")

    # 7) Live-Plot zurücksetzen
    dpg.set_value("series_live", [[], []])
    dpg.set_value("lbl_status", "▶ Messe...")

    # 8) Mess-Schleife
    run_count = 0
    recording = False
    finished = False
    xs, ys = [], []      # Für Differenzdruck groß
    xs_k, ys_k = [], []  # Für Differenzdruck klein

    while (run_count <= n_runs) and not finished:
        raw = ser.readline().decode().strip()
        print(raw)  # Debug-Ausgabe
        if raw == "Kennlinienaufnahme gestartet.":
            run_count += 1
            recording = True
            xs.clear(); ys.clear()
            xs_k.clear(); ys_k.clear()
        elif raw == "Kennlinienaufnahme beendet.":
            recording = False
            finished = True
            if run_count < n_runs:
                ser.write(b"motor record\n")
        elif recording and raw:
            parts = raw.split(",")
            print(parts)
            if len(parts) < 4:
                continue
            # Rohwerte
            v_diff_air = float(parts[0])
            v_abs      = float(parts[1])
            v_diff     = float(parts[2])
            v_flow     = float(parts[3])

            # Offset-Korrektur
            vc_air = v_diff_air - offsets["p_diff_air"]
            vc_abs = v_abs      - offsets["p_abs"]
            vc_diff= v_diff     - offsets["p_diff"]
            vc_flow= v_flow     - offsets["flow"]

            # phys. Umrechnung + Fehlerbänder
            fa_l, fa_c, fa_u = p5010_band_kpa(vc_air)
            fa_l /= 100; fa_c /= 100; fa_u /= 100
            pa_l, pa_c, pa_u = shd692_band_bar_from_v(vc_abs)
            df_l, df_c, df_u = shd692_band_bar_from_v(vc_diff)
            fl_l, fl_c, fl_u = flow_band(vc_flow)

            # Plot: Durchfluss (x) vs. Differenzdruck (y)
            xs.append(fl_c)
            ys.append(df_c)
            xs_k.append(fl_c)
            ys_k.append(fa_c)

            # Live-Plot aktualisieren (beide Linien)
            dpg.set_value("series_live", [xs[-500:], ys[-500:]])
            dpg.set_value("series_klein", [xs_k[-500:], ys_k[-500:]])

            # Zeile in CSV schreiben
            with open(csv_path,'a',newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    run_count,
                    v_flow,
                    round(fl_c,4), round(fl_l,4), round(fl_u,4),
                    v_diff_air,
                    round(fa_c,4), round(fa_l,4), round(fa_u,4),
                    v_abs, 
                    round(pa_c,4),
                    v_diff,
                    round(df_c,4), round(df_l,4), round(df_u,4)
                ])

    ser.close()
    dpg.set_value("lbl_status", "Fertig!")

def plot_selected_run(sender, app_data, user_data):
    csv_path, selected_run = user_data
    xs, ys = [], []
    xs_k, ys_k = [], []
    # Fehlerpunkte für y (Druck)
    xs_err, ys_err = [], []
    xs_k_err, ys_k_err = [], []
    # Fehlerpunkte für x (Flow)
    xs_xerr, ys_xerr = [], []
    xs_k_xerr, ys_k_xerr = [], []
    try:
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    run = int(row['Run'])
                    if selected_run != "Alle" and run != int(selected_run):
                        continue
                    # Hauptwerte
                    flow_c = float(row['Flow_center (L/min)'])
                    diff_c = float(row['Differenzdruck_center (bar)'])
                    flow_l = float(row['Flow_lower (L/min)'])
                    flow_u = float(row['Flow_upper (L/min)'])
                    diff_l = float(row['Differenzdruck_lower (bar)'])
                    diff_u = float(row['Differenzdruck_upper (bar)'])
                    xs.append(flow_c)
                    ys.append(diff_c)
                    # Fehlerpunkte für y (Druck)
                    xs_err.extend([flow_c, flow_c])
                    ys_err.extend([diff_l, diff_u])
                    # Fehlerpunkte für x (Flow)
                    xs_xerr.extend([flow_l, flow_u])
                    ys_xerr.extend([diff_c, diff_c])
                    # Für den kleinen Sensor analog:
                    flow_c_k = flow_c
                    diff_c_k = float(row['p_diff_air_center (bar)'])
                    diff_l_k = float(row['p_diff_air_lower (bar)'])
                    diff_u_k = float(row['p_diff_air_upper (bar)'])
                    xs_k.append(flow_c_k)
                    ys_k.append(diff_c_k)
                    # Fehlerpunkte für y (Druck) klein
                    xs_k_err.extend([flow_c_k, flow_c_k])
                    ys_k_err.extend([diff_l_k, diff_u_k])
                    # Fehlerpunkte für x (Flow) klein
                    xs_k_xerr.extend([flow_l, flow_u])
                    ys_k_xerr.extend([diff_c_k, diff_c_k])
                except Exception:
                    continue
        dpg.set_value("series_live", [xs, ys])
        dpg.set_value("series_klein", [xs_k, ys_k])
        # Fehlerpunkte als Scatter anzeigen (y-Fehler)
        if not dpg.does_item_exist("series_live_err"):
            dpg.add_scatter_series(xs_err, ys_err, label="Diff groß Fehler y", parent="y_live", tag="series_live_err", marker=dpg.mvPlotMarker_Circle, size=4, color=(200,0,0,120), show=dpg.get_value("chk_err"))
        else:
            dpg.set_value("series_live_err", [xs_err, ys_err])
        if not dpg.does_item_exist("series_klein_err"):
            dpg.add_scatter_series(xs_k_err, ys_k_err, label="Diff klein Fehler y", parent="y_live", tag="series_klein_err", marker=dpg.mvPlotMarker_Circle, size=4, color=(0,0,200,120), show=dpg.get_value("chk_err"))
        else:
            dpg.set_value("series_klein_err", [xs_k_err, ys_k_err])
        # Fehlerpunkte als Scatter anzeigen (x-Fehler)
        if not dpg.does_item_exist("series_live_xerr"):
            dpg.add_scatter_series(xs_xerr, ys_xerr, label="Diff groß Fehler x", parent="y_live", tag="series_live_xerr", marker=dpg.mvPlotMarker_Square, size=4, color=(200,100,0,120), show=dpg.get_value("chk_err"))
        else:
            dpg.set_value("series_live_xerr", [xs_xerr, ys_xerr])
        if not dpg.does_item_exist("series_klein_xerr"):
            dpg.add_scatter_series(xs_k_xerr, ys_k_xerr, label="Diff klein Fehler x", parent="y_live", tag="series_klein_xerr", marker=dpg.mvPlotMarker_Square, size=4, color=(0,100,200,120), show=dpg.get_value("chk_err"))
        else:
            dpg.set_value("series_klein_xerr", [xs_k_xerr, ys_k_xerr])
        dpg.set_value("lbl_status", f"CSV geladen: Run {selected_run}")
    except Exception as e:
        dpg.set_value("lbl_status", f"Fehler beim Laden: {e}")

def plot_csv_file(sender, app_data):
    csv_path = app_data['file_path_name']
    runs = set()
    try:
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    runs.add(int(row['Run']))
                except Exception:
                    continue
        runs = sorted(runs)
        run_options = ["Alle"] + [str(r) for r in runs]
        # ComboBox für Run-Auswahl anzeigen
        if dpg.does_item_exist("combo_runs"):
            dpg.delete_item("combo_runs")
        dpg.add_combo(run_options, label="Run auswählen", default_value="Alle",
                      callback=lambda s, a, u: plot_selected_run(s, a, (csv_path, dpg.get_value(s))),
                      tag="combo_runs", parent="menu_window")
        # Direkt alle Runs plotten
        plot_selected_run(None, None, (csv_path, "Alle"))
    except Exception as e:
        dpg.set_value("lbl_status", f"Fehler beim Laden: {e}")

def open_csv_dialog():
    dpg.show_item("file_dialog_id")

def toggle_error_visibility(show):
    for tag in ("series_live_err", "series_klein_err", "series_live_xerr", "series_klein_xerr"):
        if dpg.does_item_exist(tag):
            dpg.configure_item(tag, show=show)

# ---------- GUI-Aufbau ----------
dpg.create_context()
dpg.create_viewport(title="Kennlinien-Recorder", width=2000, height=1500)

# Settings-Window
with dpg.window(label="Einstellungen", pos=(10,10), width=360, height=300):
    dpg.add_input_text(label="Klappen-Name",   tag="inp_flap", default_value="F1")
    dpg.add_input_int( label="Messzeit [ms]",   tag="inp_time", default_value=1000, min_value=10, max_value=10000)
    dpg.add_input_int( label="Anz. Runs",       tag="inp_runs", default_value=3, min_value=1)
    dpg.add_checkbox(label="Offset-Kalibrierung", tag="chk_kal", default_value=False)
    dpg.add_separator()
    dpg.add_input_text(label="Port",            tag="inp_port", default_value="/dev/ttyUSB0")
    dpg.add_input_int( label="Baudrate",        tag="inp_baud", default_value=115200)
    dpg.add_button(label="Start Messung", callback=lambda s,a: run_measurement_blocking())
    dpg.add_text("", tag="lbl_status")

# Plot-Window
with dpg.window(label="Live-Plot", pos=(380,10), width=1500, height=800):
    with dpg.plot(label="Messung", tag="plot_live", height=-1, width=-1):
        dpg.add_plot_legend()
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Flow [L/min]", tag="x_live")
        with dpg.plot_axis(dpg.mvYAxis, label="Differenzdruck [bar]", tag="y_live"):
            dpg.add_line_series([], [], label="Diff groß", tag="series_live", parent="y_live")
            dpg.add_line_series([], [], label="Diff klein", tag="series_klein", parent="y_live")

# Achsenlimits setzen (nach dem Aufbau)
dpg.set_axis_limits("x_live", -0.05, 20)
dpg.set_axis_limits("y_live", -0.05, 0.15)

with dpg.window(label="Menü", pos=(10,320), width=360, height=120, tag="menu_window"):
    dpg.add_button(label="CSV laden & anzeigen", callback=lambda: open_csv_dialog())
    dpg.add_checkbox(label="Fehlerpunkte anzeigen", tag="chk_err", default_value=True,
                     callback=lambda s,a: toggle_error_visibility(a))

with dpg.file_dialog(directory_selector=False, show=False, callback=plot_csv_file, id="file_dialog_id", width=700, height=400):
    dpg.add_file_extension(".csv", color=(0, 255, 0, 255))
    dpg.add_file_extension("", color=(255, 255, 255, 255))

dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
