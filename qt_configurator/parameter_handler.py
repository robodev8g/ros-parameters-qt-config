

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem

from typing import List
from zenoh_utils import ParameterInfo, ParameterValue

PARAMETER_TYPES = {
                    "1": "bool",
                    "2": "integer",
                    "3": "double",
                    "4": "string",
                    "5": "byte array",
                    "6": "bool array",
                    "7": "integer array",
                    "8": "double array",
                    "9": "string array"
                }

# TODO: Create global variables for NAME_COL_INDEX, TYPE_COL_INDEX, VALUE_COL_INDEX


class ParameterHandler:
    def __init__(self, node_names) -> None:
        # TODO: Handle a database of node parameters with types and values (use data as cache and add refreash button)
        self.param_database = {}
        for name in node_names:
            self.param_database[name] = []

    def load_params(self, params_table: QTableWidget, node_name, parameters_info: List[ParameterInfo]):
        params_table.setRowCount(len(parameters_info))
        # Insert data to table
        for index, param_info in enumerate(parameters_info):
            self.param_database[node_name].append(param_info)
            params_table.setItem(index, 0, QTableWidgetItem(param_info.param_name))
            params_table.setItem(index, 1, QTableWidgetItem(PARAMETER_TYPES[str(param_info.param_type)]))
            params_table.setItem(index, 2, QTableWidgetItem(param_info.param_value))
        
        # Disable edit option to name, type columns (only value column is editable)
        for i in range(params_table.rowCount()):
            flags = params_table.item(i, 0).flags()
            params_table.item(i,0).setFlags(flags & ~Qt.ItemIsEditable)
            params_table.item(i,1).setFlags(flags & ~Qt.ItemIsEditable)

    def create_param_value_object(self, node_name, param_name, param_new_value) -> ParameterValue:
        for param_info in self.param_database[node_name]:
            if param_info.param_name == param_name:
                attribute_name = PARAMETER_TYPES[str(param_info.param_type)] + " value"
                attribute_name = attribute_name.replace(" ", "_")
                if "string" not in attribute_name:
                    param_new_value = eval(param_new_value)
                param_value_obj = ParameterValue(type=param_info.param_type)
                exec(f"param_value_obj.{attribute_name} = param_new_value")
                return param_value_obj
        return None
