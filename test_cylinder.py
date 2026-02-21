import sys
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)
w = gl.GLViewWidget()
w.show()

# Test cylinder
try:
    md = gl.MeshData.cylinder(rows=10, cols=20, radius=[10.0, 10.0], length=60.0)
    mesh = gl.GLMeshItem(meshdata=md, smooth=True, color=(0.2, 0.2, 0.2, 1.0), shader='shaded')
    w.addItem(mesh)
    print("Success")
except Exception as e:
    print(f"Error: {e}")

sys.exit(0)
