import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QTableWidget, QComboBox, QHeaderView, QTableWidgetItem, QAbstractItemView

from zenoh_utils import ZenohOperator

from parameter_handler import ParameterHandler

PARAM_CONFIG_APP_UI = "/home/user/projects/ros-parameters-qt-config/qt_configurator/config/configurator.ui"

class ParametersConfigurator(QMainWindow):
    def __init__(self):
        super(ParametersConfigurator, self).__init__()
        
        self.init_ui()
        self.config_params_table()
        
        self.zenoh_op = ZenohOperator()
        names = self.zenoh_op.get_node_names()
        
        self.parameter_handler = ParameterHandler(names)
        
        self.comboBox.addItems(names)
        self.comboBox.activated.connect(self.combo_box_changed)
        self.tableWidget.cellChanged.connect(self.param_value_changed)
        
        # TODO: color rows by successful of set param.
        # TODO: create Update Parameters button to set all changes
        # TODO: add parameter search tool
    
    def init_ui(self):
        ui = uic.loadUi(PARAM_CONFIG_APP_UI, self)
        self.tableWidget: QTableWidget = ui.tableWidget
        self.comboBox: QComboBox = ui.comboBox
    
    def config_params_table(self):
        self.tableWidget.setMaximumWidth(700)
        self.tableWidget.setColumnWidth(0, 300)
        self.tableWidget.setColumnWidth(1, 200)
        self.tableWidget.setColumnWidth(2, 200)

        # TODO: find a way to configure that with QtDesigner
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)


    def combo_box_changed(self, index):
        self.tableWidget.cellChanged.disconnect(self.param_value_changed)
        node_name = self.comboBox.currentText()
        parameters_info = self.zenoh_op.get_node_custom_parameter_list(node_name)
        self.parameter_handler.load_params(self.tableWidget, node_name, parameters_info)
        self.tableWidget.cellChanged.connect(self.param_value_changed)


    def param_value_changed(self, row, column):
        self.tableWidget.cellChanged.disconnect(self.param_value_changed)
        node_name = self.comboBox.currentText()
        param_name = self.tableWidget.item(row, 0).text()
        param_new_value = self.tableWidget.item(row, column).text()
        print(f"Cell ({row}, {column}) edited to: {param_new_value}")

        
        parameter_value_obj = self.parameter_handler.create_param_value_object(node_name, param_name, param_new_value)
        self.zenoh_op.send_set_parameter_req(node_name, param_name, parameter_value_obj)
        
        # TODO: where do we validate the new param value? the validity will determine the row color
        # Change row color example - color should be corrospond to success of set service
        self.tableWidget.item(row,0).setBackground(QColor(128, 255, 128))
        self.tableWidget.item(row,1).setBackground(QColor(128, 255, 128))
        self.tableWidget.item(row,2).setBackground(QColor(128, 255, 128))
        
        self.tableWidget.cellChanged.connect(self.param_value_changed)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    window = ParametersConfigurator()

    window.show()
    sys.exit(app.exec_())