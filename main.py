
import sys
import numpy as np
from kinematics.inverse import inverse_kinematics
from kinematics.jacobian import InvJac
from control.sliding_pi import SlidingPIController
from config.parameters import DT

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont
from gui.screen.MainWindow import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Fuente general
    font = QFont("Segoe UI", 10)
    app.setFont(font)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()