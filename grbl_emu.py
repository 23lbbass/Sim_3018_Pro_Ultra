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
        self.is_relative = False
        self.wco = [0.0, 0.0, 0.0]
        
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
                elif char == '~': # Cycle Start / Resume
                    if self.state == "Hold":
                        if hasattr(self, 'motion_queue') and len(self.motion_queue) > 0:
                            self.state = "Run"
                        else:
                            self.state = "Idle"
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
        status_str = f"<{self.state}|MPos:{self.mpos[0]:.3f},{self.mpos[1]:.3f},{self.mpos[2]:.3f}|WCO:{self.wco[0]:.3f},{self.wco[1]:.3f},{self.wco[2]:.3f}|FS:{int(self.feed_rate)},0>\r\n"
        self._send(status_str)

    def _handle_command(self, cmd):
        # print("RECV:", cmd)
        cmd = cmd.upper()
        
        if self.state == "Alarm":
            if cmd not in ("$X", "$H", "$HA", "$$", "?", "\x18") and not cmd.startswith("$I") and not cmd.startswith("$G"):
                self._send("error:9\r\n")
                return

        if cmd == "$$":
            for k, v in self.settings.items():
                self._send(f"{k}={v}\r\n")
            self._send("ok\r\n")
        elif cmd.startswith("$"):
            if cmd == "$H" or cmd == "$HA":
                self._start_homing()
                self._send("ok\r\n")
            elif cmd.startswith("$J="):
                self._parse_gcode(cmd[3:], is_jog=True)
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
        else:
            self._parse_gcode(cmd)
            self._send("ok\r\n")

    def _parse_gcode(self, gcode, is_jog=False):
        # Strip out comments
        gcode = re.sub(r'\(.*?\)', '', gcode)
        
        # Improved parser for linear moves and jog commands
        tokens = re.findall(r'([A-Z])([-+]?\d*\.?\d+)', gcode.upper())
        
        # Check if it's an offset/zeroing command (G92 or G10)
        for k, v in tokens:
            if k == 'G' and float(v) in (10, 92):
                for axis, val_str in tokens:
                    if axis == 'X': self.wco[0] = self.mpos[0] - float(val_str)
                    elif axis == 'Y': self.wco[1] = self.mpos[1] - float(val_str)
                    elif axis == 'Z': self.wco[2] = self.mpos[2] - float(val_str)
                return

        # Determine modal command for this block (G0, G1, G2, G3)
        # Default to whatever the last motion command was (we'll implement basic state)
        if not hasattr(self, 'motion_mode'):
            self.motion_mode = 0  # 0=G0, 1=G1, 2=G2, 3=G3

        for k, v in tokens:
            if k == 'G':
                val = int(float(v))
                if val in (0, 1, 2, 3):
                    self.motion_mode = val
        
        target = list(self.target_mpos)
        arc_offsets = {'I': 0.0, 'J': 0.0, 'K': 0.0}
        
        # Check for absolute/relative mode in this block
        is_rel_block = self.is_relative
        for k, v in tokens:
            if k == 'G':
                val = float(v)
                if val == 90:
                    is_rel_block = False
                    if not is_jog: self.is_relative = False
                elif val == 91:
                    is_rel_block = True
                    if not is_jog: self.is_relative = True
        
        is_move = False
        for k, v in tokens:
            val = float(v)
            if k == 'F':
                self.feed_rate = max(val, 0.1)
            elif k == 'X':
                target[0] = self.target_mpos[0] + val if is_rel_block else val + self.wco[0]
                is_move = True
            elif k == 'Y':
                target[1] = self.target_mpos[1] + val if is_rel_block else val + self.wco[1]
                is_move = True
            elif k == 'Z':
                target[2] = self.target_mpos[2] + val if is_rel_block else val + self.wco[2]
                is_move = True
            elif k in ('I', 'J', 'K'):
                arc_offsets[k] = val
            elif k == 'M':
                # M0 = Hold, M3/M4 = Spindle On, M5 = Spindle Off, M30 = End
                m_val = int(val)
                if m_val == 0:
                    self.state = "Hold"
                    # We would technically wait for a '~' resume command 
                elif m_val in (3, 4):
                    # Spindle on
                    pass 
                elif m_val == 5:
                    # Spindle off
                    pass
                elif m_val == 30:
                    # End of program, technically rewinds
                    pass
                
        if is_move:
            if not hasattr(self, 'motion_queue'):
                self.motion_queue = []
                
            if self.motion_mode in (2, 3):
                # Generate arc segments
                self._generate_arc(target, arc_offsets, self.motion_mode == 2)
            else:
                self.motion_queue.append(target)
                
            self.target_mpos = self.motion_queue[0]
            self.state = "Run"

    def _generate_arc(self, target, offsets, is_cw):
        # Extremely simplified arc handling, assumes G17 (XY plane)
        start_x = self.target_mpos[0]
        start_y = self.target_mpos[1]
        
        center_x = start_x + offsets['I']
        center_y = start_y + offsets['J']
        
        end_x = target[0]
        end_y = target[1]
        
        radius = math.hypot(start_x - center_x, start_y - center_y)
        
        if radius < 0.001:
            self.motion_queue.append(target)
            return
            
        start_angle = math.atan2(start_y - center_y, start_x - center_x)
        end_angle = math.atan2(end_y - center_y, end_x - center_x)
        
        # Ensure correct direction traversal
        if is_cw:
            while end_angle > start_angle:
                end_angle -= 2 * math.pi
        else:
            while end_angle < start_angle:
                end_angle += 2 * math.pi
                
        # If full circle
        if abs(end_angle - start_angle) < 0.0001:
            if is_cw:
                end_angle = start_angle - 2 * math.pi
            else:
                end_angle = start_angle + 2 * math.pi

        angular_travel = abs(end_angle - start_angle)
        
        # Segment length approx 1mm
        num_segments = max(2, int(radius * angular_travel / 1.0))
        
        z_start = self.target_mpos[2]
        z_end = target[2]
        
        for i in range(1, num_segments + 1):
            t = i / num_segments
            angle = start_angle + (end_angle - start_angle) * t
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = z_start + (z_end - z_start) * t
            
            self.motion_queue.append([x, y, z])
            
        self.target_mpos = target # update logical target for next block

    def _start_homing(self):
        self.state = "Home"
        self.homing = True
        self.home_pos = [self.settings["$130"], self.settings["$131"], self.settings["$132"]]
        
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
            self.target_mpos = list(self.home_pos)
            if hasattr(self, 'motion_queue'):
                self.motion_queue.clear()
            self.homing = False
            self.state = "Idle"
            
        threading.Thread(target=run_homing, daemon=True).start()

    def _motion_loop(self):
        last_time = time.time()
        if not hasattr(self, 'motion_queue'):
            self.motion_queue = []
            
        while self.running:
            now = time.time()
            dt = now - last_time
            last_time = now
            
            if self.state in ("Run", "Home"):
                if self.state == "Run" and hasattr(self, 'motion_queue') and len(self.motion_queue) > 0:
                    current_target = self.motion_queue[0]
                else:
                    current_target = self.target_mpos
                    
                dx = current_target[0] - self.mpos[0]
                dy = current_target[1] - self.mpos[1]
                dz = current_target[2] - self.mpos[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > 0.001:
                    # In Home state we hardcode a modest speed, otherwise feed_rate
                    speed = 500.0 if self.state == "Home" else max(self.feed_rate, 100)
                    step = (speed / 60.0) * dt
                    
                    if step >= dist:
                        next_pos = list(current_target)
                    else:
                        next_pos = [
                            self.mpos[0] + (dx / dist) * step,
                            self.mpos[1] + (dy / dist) * step,
                            self.mpos[2] + (dz / dist) * step
                        ]
                        
                    hit_limit = False
                    max_pos = [self.settings["$130"], self.settings["$131"], self.settings["$132"]]
                    for i in range(3):
                        if next_pos[i] < 0.0 or next_pos[i] > max_pos[i]:
                            hit_limit = True
                            next_pos[i] = max(0.0, min(next_pos[i], max_pos[i]))
                    
                    self.mpos = next_pos
                    
                    if hit_limit and not self.homing:
                        self.target_mpos = list(self.mpos)
                        if hasattr(self, 'motion_queue'):
                            self.motion_queue.clear()
                        self.state = "Alarm"
                        self._send("ALARM:1\r\n[MSG:Hard limit triggered. Machine position is likely lost due to sudden and immediate halt. Re-homing is highly recommended.]\r\n")
                    elif step >= dist:
                        if self.state == "Run" and hasattr(self, 'motion_queue') and len(self.motion_queue) > 0:
                            self.motion_queue.pop(0)
                        if not self.homing and not (hasattr(self, 'motion_queue') and len(self.motion_queue) > 0):
                            self.state = "Idle"
                else:
                    if self.state == "Run" and hasattr(self, 'motion_queue') and len(self.motion_queue) > 0:
                        self.motion_queue.pop(0)
                    if not self.homing and not (hasattr(self, 'motion_queue') and len(self.motion_queue) > 0):
                        self.state = "Idle"
            
            time.sleep(0.01)

    def close(self):
        self.running = False
        os.close(self.master)
        os.close(self.slave)
