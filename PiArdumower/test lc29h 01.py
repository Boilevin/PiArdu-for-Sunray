import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from gnss_manager import GNSSManager

def parse_gga(nmea):
    try:
        parts = nmea.split(",")
        if parts[0].endswith("GGA") and len(parts) > 5:
            lat = float(parts[2])
            lon = float(parts[4])
            return lat, lon
    except:
        pass
    return None, None

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Rover GNSS Position")
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)
        self.lat_data = []
        self.lon_data = []
        self.point, = self.ax.plot([], [], 'ro')
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_title("Position du rover")
        self.ax.grid(True)
        self.ax.set_xlim(0.9995, 1.0005)
        self.ax.set_ylim(44.9995, 45.0005)

    def update_position(self, nmea):
        lat, lon = parse_gga(nmea)
        if lat and lon:
            self.lat_data.append(lat)
            self.lon_data.append(lon)
            self.point.set_data(self.lon_data, self.lat_data)
            self.ax.set_xlim(min(self.lon_data)-0.0002, max(self.lon_data)+0.0002)
            self.ax.set_ylim(min(self.lat_data)-0.0002, max(self.lat_data)+0.0002)
            self.canvas.draw_idle()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    gnss = GNSSManager()
    gnss.set_nmea_callback(lambda nmea: root.after(0, app.update_position, nmea))
    gnss.start_rtcm_thread()
    root.mainloop()