import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from pyproj import Proj
import threading
import time
import random
from gnss_manager import GNSSManager
import math



# --- Parsing NMEA GGA ---
def nmea_to_decimal(coord, direction):
    if not coord or coord == '':
        return None
    deg = int(float(coord) // 100)
    minutes = float(coord) - deg * 100
    decimal = deg + minutes / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def parse_gga(nmea):
    try:
        parts = nmea.split(",")
        #print("Parts GGA:", parts)
        if not (parts[0].endswith("GGA") and len(parts) > 9):
            return None, None, None
        if parts[2] == '' or parts[3] == '' or parts[4] == '' or parts[5] == '' or parts[9] == '':
            print("Champ vide dans GGA")
            return None, None, None
        lat = nmea_to_decimal(parts[2], parts[3])
        lon = nmea_to_decimal(parts[4], parts[5])
        alt = float(parts[9])
        #print(f"lat={lat}, lon={lon}, alt={alt}")
        return lat, lon, alt
    except Exception as e:
        print("Erreur parsing GGA:", e, "| NMEA:", nmea)
    return None, None, None

def get_gps_fix_status_from_gga(nmea_line):
    if nmea_line.startswith("$GNGGA") or nmea_line.startswith("$GPGGA"):
        parts = nmea_line.strip().split(",")
        if len(parts) > 6:
            fix_quality = parts[6]
            if fix_quality == "1":
                return "SPS"
            elif fix_quality == "2":
                return "DGPS"
            elif fix_quality == "4":
                return "RTK FIX"
            elif fix_quality == "5":
                return "RTK FLOAT"
            else:
                return "INVALIDE"
    return None

def get_fix_color(status):
    if status == "SPS":
        return "blue"
    elif status == "DGPS":
        return "red"
    elif status == "RTK FIX":
        return "green"
    elif status == "RTK FLOAT":
        return "orange"
    else:
        return "gray"

# --- Base RTK ---
BASE_LON = 0.00429279
BASE_LAT = 45.13401093
BASE_ALT = 97.91

# --- UTM projection (zone 31N pour la France) ---
proj = Proj(proj='utm', zone=31, ellps='WGS84', south=False)
base_x, base_y = proj(BASE_LON, BASE_LAT)

# --- Interface graphique ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Rover GNSS Position (relatif à la base)")
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
        self.lat_data = []
        self.lon_data = []
        self.x_data = []
        self.y_data = []
        self.points, = self.ax.plot([], [], 'o', markersize=5)  # Pour toutes les positions
        self.current_point, = self.ax.plot([], [], 'o', markersize=10)  # Pour la position courante
        self.ax.set_xlabel("ΔX (cm)")
        self.ax.set_ylabel("ΔY (cm)")
        self.ax.set_title("Position du rover (relatif à la base)")
        self.ax.grid(True)
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)

        # Label pour afficher la position relative
        self.label_rel = tk.Label(root, text="ΔX=0.00 m, ΔY=0.00 m, ΔZ=0.00 m", font=("Arial", 14), fg="blue")
        self.label_rel.pack(pady=10)

        # Zone de texte pour afficher les messages de débogage
        self.text_console = tk.Text(root, height=10, width=80)
        self.text_console.pack(pady=5)

    def update_position(self, nmea):
        #self.insert_console(nmea)  # Affiche chaque trame reçue
        if not nmea.startswith("$GNGGA") and not nmea.startswith("$GPGGA"):
            return  # Ignore les trames non-GGA
        lat, lon, alt = parse_gga(nmea)
        status = get_gps_fix_status_from_gga(nmea)
        if lat is not None and lon is not None and alt is not None:
            rover_x, rover_y = proj(lon, lat)
            dx = (rover_x - base_x) * 100  # en cm
            dy = (rover_y - base_y) * 100  # en cm

            # Ajoute la nouvelle position aux listes
            self.x_data.append(dx)
            self.y_data.append(dy)

            # Affiche toutes les positions précédentes en gris
            self.points.set_data(self.x_data, self.y_data)
            self.points.set_color("gray")

            # Affiche la position courante dans la bonne couleur
            color = get_fix_color(status)
            self.current_point.set_data([dx], [dy])
            self.current_point.set_color(color)

            margin = 50  # cm
            self.ax.set_xlim(dx - margin, dx + margin)
            self.ax.set_ylim(dy - margin, dy + margin)
            self.ax.set_xlabel("ΔX (cm)")
            self.ax.set_ylabel("ΔY (cm)")
            self.canvas.draw_idle()
            self.label_rel.config(text=f"ΔX={dx:.1f} cm, ΔY={dy:.1f} cm, ΔZ={(alt-BASE_ALT)*100:.1f} cm")
        else:
            print("Coordonnées invalides, pas d'affichage.")

    def insert_console(self, texte):
        # Ajoute le texte en haut de la TextBox
        self.text_console.insert('1.0', texte)
        # Optionnel : limite la taille de la console
        if float(self.text_console.index('end')) > 200.0:
            self.text_console.delete('200.0', 'end')

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    gnss = GNSSManager()
    gnss.set_nmea_callback(lambda nmea: root.after(0, app.update_position, nmea))
    gnss.start_rtcm_thread(consoleInsertText=lambda txt: root.after(0, app.insert_console, txt))
    root.mainloop()