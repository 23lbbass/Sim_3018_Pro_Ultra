import sys
import numpy as np
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QSizePolicy
from PyQt5.QtCore import QTimer
from pyqtgraph.opengl import GLMeshItem, MeshData

def create_box_meshdata(size):
    l, w, h = size
    verts = np.array([
        [0, 0, 0], [l, 0, 0], [l, w, 0], [0, w, 0],
        [0, 0, h], [l, 0, h], [l, w, h], [0, w, h],
        [0, 0, 0], [l, 0, 0], [l, 0, h], [0, 0, h],
        [0, w, 0], [l, w, 0], [l, w, h], [0, w, h],
        [0, 0, 0], [0, w, 0], [0, w, h], [0, 0, h],
        [l, 0, 0], [l, w, 0], [l, w, h], [l, 0, h],
    ], dtype=np.float32)
    faces = np.array([
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [8, 9, 10], [8, 10, 11], [12, 14, 13], [12, 15, 14],
        [16, 17, 18], [16, 18, 19], [20, 22, 21], [20, 23, 22],
    ], dtype=np.uint32)
    return MeshData(vertexes=verts, faces=faces)

class CNCGui(QMainWindow):
    def __init__(self, emulator=None):
        super().__init__()
        self.emulator = emulator
        self.setWindowTitle("3018 Pro Ultra Simulator")
        self.resize(1200, 800)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px; background-color: #f0f0f0; padding: 10px; border-bottom: 2px solid #ccc;")
        self.status_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        self.layout.addWidget(self.status_label)
        
        self.view = gl.GLViewWidget()
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view.setBackgroundColor('#E5E7EB')
        self.layout.addWidget(self.view)
        
        # Look from the front-left!
        self.view.setCameraPosition(distance=600, elevation=35, azimuth=-140)
        self.view.pan(150, 140, 50)
        
        grid = gl.GLGridItem()
        grid.setSize(x=500, y=500, z=0)
        grid.setSpacing(x=10, y=10, z=10)
        grid.translate(150, 140, -40)
        self.view.addItem(grid)
        
        self.color_orange = (1.0, 0.55, 0.0, 1.0)
        self.color_silver = (0.7, 0.7, 0.75, 1.0)
        self.color_dark = (0.2, 0.2, 0.2, 1.0)
        self.color_motor = (0.1, 0.1, 0.1, 1.0)
        self.color_table = (0.9, 0.9, 0.9, 1.0)
        
        self.y_table_items = []
        self.x_carriage_items = []
        self.z_assembly_items = []
        
        self.build_machine()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_state)
        self.update_timer.start(30)

    def create_box(self, size, color, group=None, offset=(0,0,0)):
        md = create_box_meshdata(size)
        mesh = GLMeshItem(meshdata=md, smooth=False, color=color, shader='shaded')
        if group is not None:
            group.append((mesh, offset, None))
        else:
            mesh.translate(*offset)
            self.view.addItem(mesh)
        return mesh

    def create_cylinder(self, radius, length, color, group=None, offset=(0,0,0), rotate=None):
        md = MeshData.cylinder(rows=15, cols=20, radius=[radius, radius], length=length)
        mesh = GLMeshItem(meshdata=md, smooth=True, color=color, shader='shaded')
        if group is not None:
            group.append((mesh, offset, rotate))
        else:
            if rotate:
                mesh.rotate(*rotate)
            mesh.translate(*offset)
            self.view.addItem(mesh)
        return mesh

    def build_machine(self):
        b_w, b_l = 340, 280
        x_start, y_start = -20, 0
        tool_y = 170 # Y position of the spindle in world coordinates
        
        # 1. Base (Fixed)
        self.create_box((20, b_l, 40), self.color_orange, offset=(x_start, y_start, -40))
        self.create_box((20, b_l, 40), self.color_orange, offset=(x_start + b_w - 20, y_start, -40))
        # Front and back cross-members (Silver, 20x40 upright)
        self.create_box((b_w-40, 20, 40), self.color_silver, offset=(x_start+20, y_start, -40))
        self.create_box((b_w-40, 20, 40), self.color_silver, offset=(x_start+20, y_start + b_l - 20, -40))
        
        # Y-Axis Rods and Leadscrew (+Y direction)
        self.create_cylinder(6, b_l, self.color_silver, offset=(x_start + 60, y_start, -20), rotate=(-90, 1, 0, 0))
        self.create_cylinder(6, b_l, self.color_silver, offset=(x_start + b_w - 60, y_start, -20), rotate=(-90, 1, 0, 0))
        self.create_cylinder(4, b_l, self.color_silver, offset=(x_start + (b_w/2), y_start, -20), rotate=(-90, 1, 0, 0))
        
        # Y Motor (Back of base, extending into +Y)
        self.create_box((40, 40, 40), self.color_motor, offset=(x_start + (b_w/2) - 20, y_start + b_l, -30))

        # 2. X-Gantry (Fixed)
        g_y = tool_y + 40 # Uprights just behind tool
        self.create_box((20, 30, 200), self.color_orange, offset=(x_start, g_y, 0))
        self.create_box((20, 30, 200), self.color_orange, offset=(x_start + b_w - 20, g_y, 0))
        self.create_box((b_w-40, 30, 80), self.color_silver, offset=(x_start+20, g_y - 20, 80))
        
        # X Motor (Right side)
        self.create_box((40, 40, 40), self.color_motor, offset=(x_start + b_w, g_y - 15, 100))
        # X Leadscrew (+X direction)
        self.create_cylinder(4, b_w, self.color_silver, offset=(x_start, g_y - 5, 120), rotate=(90, 0, 1, 0))

        # 3. Y-Table (Moves along Y)
        t_w, t_l = 300, 180
        table_start_y = 170
        self.create_box((t_w, t_l, 15), self.color_table, group=self.y_table_items, offset=(0, table_start_y, -15))
        for i in range(1, 6):
            self.create_box((t_w, 2, 2), self.color_dark, group=self.y_table_items, offset=(0, table_start_y + i*30, -2))

        # 4. X-Carriage (Moves along X)
        c_w, c_h = 70, 90
        # Carriage assembly slightly behind tool
        self.create_box((c_w, 30, c_h), self.color_orange, group=self.x_carriage_items, offset=(-c_w/2, tool_y + 15, 75))
        self.create_box((40, 40, 40), self.color_motor, group=self.x_carriage_items, offset=(-20, tool_y + 10, 75 + c_h))
        self.create_cylinder(3, c_h, self.color_silver, group=self.x_carriage_items, offset=(0, tool_y + 30, 75))

        # 5. Z-Assembly (Moves along Z)
        self.create_box((50, 30, 60), self.color_dark, group=self.z_assembly_items, offset=(-25, tool_y - 15, 40))
        self.create_cylinder(22, 100, self.color_motor, group=self.z_assembly_items, offset=(0, tool_y, 50))
        self.create_cylinder(20, 30, self.color_silver, group=self.z_assembly_items, offset=(0, tool_y, 20))
        self.create_cylinder(2, 20, self.color_silver, group=self.z_assembly_items, offset=(0, tool_y, 0))
        
        for item in self.y_table_items: self.view.addItem(item[0])
        for item in self.x_carriage_items: self.view.addItem(item[0])
        for item in self.z_assembly_items: self.view.addItem(item[0])

    def update_state(self):
        if self.emulator:
            state = self.emulator.state
            mpos = self.emulator.mpos
            port = self.emulator.port_name
            
            self.status_label.setText(
                f"State: {state} | Port: {port} | MPos: X:{mpos[0]:.2f} Y:{mpos[1]:.2f} Z:{mpos[2]:.2f} | Machine: 3018 Pro Ultra"
            )
            
            # The table offsets are already configured such that MPos Y=0 
            # places the front edge of the table under the tool.
            # As MPos Y increases, the table should move backwards (towards +Y).
            # Wait, Y+ moves the table forward so the tool reaches the back.
            # If MPos[1] = 180, table must move to -Y, so tool is at the back.
            table_y_world = -mpos[1]
            carriage_x_world = mpos[0]
            z_world = mpos[2]
            
            for item in self.y_table_items:
                obj, offset, rot = item
                obj.resetTransform()
                if rot: obj.rotate(*rot)
                obj.translate(offset[0], offset[1] + table_y_world, offset[2])
                
            for item in self.x_carriage_items:
                obj, offset, rot = item
                obj.resetTransform()
                if rot: obj.rotate(*rot)
                obj.translate(offset[0] + carriage_x_world, offset[1], offset[2])
                
            for item in self.z_assembly_items:
                obj, offset, rot = item
                obj.resetTransform()
                if rot: obj.rotate(*rot)
                obj.translate(offset[0] + carriage_x_world, offset[1], offset[2] + z_world)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CNCGui()
    window.show()
    sys.exit(app.exec_())
