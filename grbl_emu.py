import os
import pty
import tty
import termios
import threading
import time
import re
import math

class GrblEmulator:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port_name = os.ttyname(self.slave)
        
        # Configure the slave port to raw mode to emulate serial
        tty.setraw(self.slave)
        
        # Disable echo on the slave port
        attrs = termios.tcgetattr(self.slave)
        attrs[3] = attrs[3] & ~termios.ECHO
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)

        print(f"GRBL Emulator listening on: {self.port_name}")

        self.running = True
        self.state = "Idle"
        self.mpos = [0.0, 0.0, 0.0]  # X, Y, Z
        self.wpos = [0.0, 0.0, 0.0]
        self.target_mpos = [0.0, 0.0, 0.0]
        self.feed_rate = 1000.0  # mm/min
        
        # Homing parameters
        self.home_pos = [300.0, 180.0, 45.0]  # Max travel position (Home)
        self.homing = False
        
        # Grbl settings map
        self.settings = {
            "$0": 10, "$1": 25, "$2": 0, "$3": 0, "$4": 0, "$5": 0, "$6": 0,
            "$10": 1, "$11": 0.010, "$12": 0.002, "$13": 0, "$20": 0, "$21": 0,
            "$22": 1, "$23": 0, "$24": 25.0, "$25": 500.0, "$26": 250, "$27": 1.0,
            "$30": 1000, "$31": 0, "$32": 0,
            "$100": 800.0, "$101": 800.0, "$102": 800.0,
            "$110": 2000.0, "$111": 2000.0, "$112": 2000.0,
            "$120": 20.0, "$121": 20.0, "$122": 20.0,
            "$130": 300.0, "$131": 180.0, "$132": 45.0
        }

        self.thread = threading.Thread(target=self._serial_loop)
        self.thread.daemon = True
        self.thread.start()

        self.move_thread = threading.Thread(target=self._motion_loop)
        self.move_thread.daemon = True
        self.move_thread.start()

    def _serial_loop(self):
        buffer = ""
        while self.running:
            try:
                # Read 1 byte at a time
                data = os.read(self.master, 1)
                if not data:
                    break
                
                char = data.decode("utf-8", errors="ignore")
                
                if char == '?':
                    self._send_status()
                elif char == '\x18': # Ctrl-X Soft Reset
                    self.state = "Alarm"
                    self._send("Grbl 1.1f ['$' for help]\r\n")
                elif char in ('\r', '\n'):
                    if buffer.strip():
                        self._handle_command(buffer.strip())
                    buffer = ""
                else:
                    buffer += char
            except OSError:
                break
            except Exception as e:
                print(f"Serial Error: {e}")

    def _send(self, msg):
        try:
            os.write(self.master, msg.encode("utf-8"))
        except:
            pass

    def _send_status(self):
        # Format: <Idle|MPos:0.000,0.000,0.000|Bf:15,128|FS:0,0>
        status_str = f"<{self.state}|MPos:{self.mpos[0]:.3f},{self.mpos[1]:.3f},{self.mpos[2]:.3f}|FS:{int(self.feed_rate)},0>\r\n"
        self._send(status_str)

    def _handle_command(self, cmd):
        # print("RECV:", cmd)
        cmd = cmd.upper()
        if cmd == "$$":
            for k, v in self.settings.items():
                self._send(f"{k}={v}\r\n")
            self._send("ok\r\n")
        elif cmd.startswith("$"):
            if cmd == "$H" or cmd == "$HA":
                self._start_homing()
                self._send("ok\r\n")
            elif cmd == "$I":
                self._send("[VER:1.1f.20170801:]\r\n[OPT:V,15,128]\r\nok\r\n")
            elif cmd == "$X":
                self.state = "Idle"
                self._send("ok\r\n")
            elif cmd == "$G":
                self._send("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\nok\r\n")
            else:
                self._send("ok\r\n") # Fake setting success
        elif cmd.startswith("G"):
            self._parse_gcode(cmd)
            self._send("ok\r\n")
        else:
            self._send("ok\r\n")

    def _parse_gcode(self, gcode):
        # Very basic GCode parser for linear moves (G0, G1)
        tokens = gcode.split()
        is_move = False
        target = list(self.target_mpos)
        
        for token in tokens:
            if token.startswith('F'):
                self.feed_rate = float(token[1:])
            elif token.startswith('X'):
                target[0] = float(token[1:])
                is_move = True
            elif token.startswith('Y'):
                target[1] = float(token[1:])
                is_move = True
            elif token.startswith('Z'):
                target[2] = float(token[1:])
                is_move = True
                
        if is_move:
            self.target_mpos = target
            self.state = "Run"

    def _start_homing(self):
        self.state = "Home"
        self.homing = True
        
        def run_homing():
            # Z up to limit
            self.target_mpos[2] = self.home_pos[2]
            while abs(self.mpos[2] - self.home_pos[2]) > 0.1 and self.homing:
                time.sleep(0.01)
                
            # Back off slightly
            self.target_mpos[2] = self.home_pos[2] - 2.0
            time.sleep(0.5)
            self.mpos[2] = self.target_mpos[2]
            
            # Slow return
            self.target_mpos[2] = self.home_pos[2]
            while abs(self.mpos[2] - self.home_pos[2]) > 0.1 and self.homing:
                time.sleep(0.01)
                
            # Now X and Y together
            self.target_mpos[0] = self.home_pos[0]
            self.target_mpos[1] = self.home_pos[1]
            while (abs(self.mpos[0] - self.home_pos[0]) > 0.1 or 
                   abs(self.mpos[1] - self.home_pos[1]) > 0.1) and self.homing:
                time.sleep(0.01)
                
            # Back off X, Y
            self.target_mpos[0] = self.home_pos[0] - 2.0
            self.target_mpos[1] = self.home_pos[1] - 2.0
            time.sleep(0.5)
            self.mpos[0] = self.target_mpos[0]
            self.mpos[1] = self.target_mpos[1]
            
            # Slow return X, Y
            self.target_mpos[0] = self.home_pos[0]
            self.target_mpos[1] = self.home_pos[1]
            while (abs(self.mpos[0] - self.home_pos[0]) > 0.1 or 
                   abs(self.mpos[1] - self.home_pos[1]) > 0.1) and self.homing:
                time.sleep(0.01)
                
            # Finish homing
            self.mpos = list(self.home_pos)
            self.wpos = [0.0, 0.0, 0.0]
            self.homing = False
            self.state = "Idle"
            
        threading.Thread(target=run_homing, daemon=True).start()

    def _motion_loop(self):
        last_time = time.time()
        while self.running:
            now = time.time()
            dt = now - last_time
            last_time = now
            
            if self.state in ("Run", "Home"):
                dx = self.target_mpos[0] - self.mpos[0]
                dy = self.target_mpos[1] - self.mpos[1]
                dz = self.target_mpos[2] - self.mpos[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > 0.001:
                    # In Home state we hardcode a modest speed, otherwise feed_rate
                    speed = 500.0 if self.state == "Home" else max(self.feed_rate, 100)
                    step = (speed / 60.0) * dt
                    
                    if step >= dist:
                        self.mpos = list(self.target_mpos)
                        if not self.homing:
                            self.state = "Idle"
                    else:
                        self.mpos[0] += (dx / dist) * step
                        self.mpos[1] += (dy / dist) * step
                        self.mpos[2] += (dz / dist) * step
                else:
                    if not self.homing:
                        self.state = "Idle"
            
            time.sleep(0.01)

    def close(self):
        self.running = False
        os.close(self.master)
        os.close(self.slave)
