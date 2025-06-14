import serial
import threading
import socket
import time
from config import myOS

class GNSSManager:
    def __init__(self, port=None, baud=None, udp_port=5000):
        if myOS == "Linux":
            self.port = '/dev/ttyAMA1'
        else:
            self.port = port or 'COM3'
        self.baud = baud or 460800
        self.udp_port = udp_port
        self.rtcm_thread = None
        self.rtcm_thread_running = False
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1)

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_command(self, cmd):
        if self.ser:
            self.ser.write((cmd + '\r\n').encode('utf-8'))
            time.sleep(0.2)
            return self.ser.readline().decode(errors='ignore')
        return None

    def send_gnss_config_sequence(self, consoleInsertText=None):
        cmds = [
            'AT+QGNSS=0',
            'AT+QGNSSCFG="outport",2',
            'AT+QGNSSCFG="save"',
            'AT+QGNSS=1'
        ]
        responses = []
        try:
            self.open()
            for cmd in cmds:
                self.ser.write((cmd + '\r\n').encode('utf-8'))
                if consoleInsertText:
                    consoleInsertText(f"Envoyé au LC29HEA: {cmd}\n")
                resp = self.ser.readline()
                if resp and consoleInsertText:
                    try:
                        txt = resp.decode(errors='ignore')
                        consoleInsertText(f"Réponse: {txt}\n")
                    except:
                        pass
                time.sleep(0.5)
                responses.append((cmd, resp))
            self.close()
        except Exception as e:
            if consoleInsertText:
                consoleInsertText(f"Erreur envoi: {e}\n")
        return responses

    def rtcm_udp_to_lc29hea(self, consoleInsertText=None, txtConsoleRecu=None):
        try:
            self.open()
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('', self.udp_port))
            if consoleInsertText:
                consoleInsertText(f"En écoute UDP sur le port {self.udp_port}\n")

            def read_lc29hea():
                while self.rtcm_thread_running:
                    line = self.ser.readline()
                    if line:
                        try:
                            txt = line.decode(errors='ignore')
                            if consoleInsertText:
                                consoleInsertText(f"LC29HEA: {txt.strip()}\n")
                            if txtConsoleRecu:
                                try:
                                    txtConsoleRecu.insert('1.0', txt)
                                except:
                                    pass
                        except Exception as e:
                            if consoleInsertText:
                                consoleInsertText(f"Erreur décodage NMEA: {e}\n")
            threading.Thread(target=read_lc29hea, daemon=True).start()

            while self.rtcm_thread_running:
                data, addr = sock.recvfrom(1024)
                self.ser.write(data)
        except Exception as e:
            if consoleInsertText:
                consoleInsertText(f"Erreur RTCM/LC29HEA: {e}\n")
        finally:
            self.rtcm_thread_running = False
            self.close()

    def start_rtcm_thread(self, consoleInsertText=None, txtConsoleRecu=None):
        if not self.rtcm_thread_running:
            self.rtcm_thread_running = True
            self.rtcm_thread = threading.Thread(
                target=self.rtcm_udp_to_lc29hea,
                args=(consoleInsertText, txtConsoleRecu),
                daemon=True
            )
            self.rtcm_thread.start()
            if consoleInsertText:
                consoleInsertText("RTCM/LC29HEA : Démarrage du flux\n")
        else:
            if consoleInsertText:
                consoleInsertText("RTCM/LC29HEA : Déjà en cours\n")

    def stop_rtcm_thread(self, consoleInsertText=None):
        if self.rtcm_thread_running:
            self.rtcm_thread_running = False
            if consoleInsertText:
                consoleInsertText("RTCM/LC29HEA : Arrêt demandé\n")
        else:
            if consoleInsertText:
                consoleInsertText("RTCM/LC29HEA : Déjà arrêté\n")