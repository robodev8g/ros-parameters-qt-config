import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QTableWidget, QComboBox, QHeaderView, QTableWidgetItem, QAbstractItemView, QItemDelegate

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

        # TODO: find a way to configure that with QtDesigner
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)

        
        
        self.zenoh_op = ZenohOperator()
        names = self.zenoh_op.get_node_names()
        self.comboBox.addItems(names)
        self.comboBox.activated.connect(self.combo_box_changed)
        # self.zenoh_op.send_set_parameter_req()

        
        # TODO: enable table edit option only for value column
        # TODO: color rows by successful of set param.
        # TODO: create Update Parameters button to set all changes
        # TODO: add parameter search tool
        
        
        
    def combo_box_changed(self, index):
        current_value = self.comboBox.currentText()
        parameters_info = self.zenoh_op.get_node_custom_parameter_list(current_value)
        
        self.tableWidget.setRowCount(len(parameters_info))
        for index, param_info in enumerate(parameters_info):
            self.tableWidget.setItem(index, 0, QTableWidgetItem(param_info.param_name))
            self.tableWidget.setItem(index, 1, QTableWidgetItem(param_info.param_type))
            self.tableWidget.setItem(index, 2, QTableWidgetItem(param_info.param_value))
        
        
        
        for i in range(self.tableWidget.rowCount()):
            flags = self.tableWidget.item(i, 0).flags()
            self.tableWidget.item(i,0).setFlags(flags & ~Qt.ItemIsEditable)
            self.tableWidget.item(i,1).setFlags(flags & ~Qt.ItemIsEditable)
        
        # Change row color example
        # self.tableWidget.item(0,0).setBackground(QColor(128, 255, 128))
        # self.tableWidget.item(0,1).setBackground(QColor(128, 255, 128))
        # self.tableWidget.item(0,2).setBackground(QColor(128, 255, 128))

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