import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import Qt

from zenoh_utils import ZenohOperator

class ParametersConfigurator(QMainWindow):
    def __init__(self):
        super(ParametersConfigurator, self).__init__()
        
        self.ui = uic.loadUi("/home/user/projects/ros-parameters-qt-config/qt_configurator/config/configurator.ui", self)
        
        # self.zenoh_op = ZenohOperator()
        # self.zenoh_op.send_set_parameter_req()
        # self.zenoh_op.get_node_parameters_list()
        


    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    
    window = ParametersConfigurator()

    window.show()
    sys.exit(app.exec_())