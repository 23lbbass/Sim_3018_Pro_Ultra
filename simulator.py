import sys
import signal
from PyQt5.QtWidgets import QApplication
from grbl_emu import GrblEmulator
from gui import CNCGui

def main():
    # Allow simple keyboard interrupt (Ctrl+C)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Initialize the Emulator
    print("Starting 3018 Simulator Emulator...")
    emu = GrblEmulator()

    # Initialize the GUI
    print("Starting 3018 GUI...")
    app = QApplication(sys.argv)
    window = CNCGui(emulator=emu)
    window.show()

    # Run the Qt Event Loop
    exit_code = app.exec_()
    
    # Cleanup
    print("Shutting down emulator...")
    emu.close()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
