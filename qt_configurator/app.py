import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QTableWidget, QComboBox, QHeaderView, QTableWidgetItem

from zenoh_utils import ZenohOperator

class ParametersConfigurator(QMainWindow):
    def __init__(self):
        super(ParametersConfigurator, self).__init__()
        
        self.ui = uic.loadUi("/home/user/projects/ros-parameters-qt-config/qt_configurator/config/configurator.ui", self)
        self.init_ui()
        self.tableWidget.setMaximumWidth(700)
        self.tableWidget.setColumnWidth(0, 300)
        self.tableWidget.setColumnWidth(1, 200)
        self.tableWidget.setColumnWidth(2, 200)

        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.zenoh_op = ZenohOperator()
        names = self.zenoh_op.get_node_names()
        self.comboBox.addItems(names)
        self.comboBox.activated.connect(self.combo_box_changed)
        # self.zenoh_op.send_set_parameter_req()

        
        # TODO: clock table edit option (the use should edit only the value column)
        # TODO: add parameter search tool
        
        
        
    def combo_box_changed(self, index):
        current_value = self.comboBox.currentText()
        parameters_info = self.zenoh_op.get_node_custom_parameter_list(current_value)
        
        self.tableWidget.setRowCount(len(parameters_info))
        for index, param_info in enumerate(parameters_info):
            self.tableWidget.setItem(index, 0, QTableWidgetItem(param_info.param_name))
            self.tableWidget.setItem(index, 1, QTableWidgetItem(param_info.param_type))
            self.tableWidget.setItem(index, 2, QTableWidgetItem(param_info.param_value))
        
    

    def init_ui(self):
        self.tableWidget: QTableWidget = self.ui.tableWidget
        self.comboBox: QComboBox = self.ui.comboBox
        


    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    
    window = ParametersConfigurator()

    window.show()
    sys.exit(app.exec_())