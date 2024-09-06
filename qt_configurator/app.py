import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5 import uic
from PyQt5.QtCore import Qt

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("/home/user/projects/ros-parameters-qt-config/qt_configurator/configurator.ui")
        

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWidget()

    window.show()
    sys.exit(app.exec_())