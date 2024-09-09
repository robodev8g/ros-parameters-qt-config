import zenoh

from dataclasses import dataclass, field
from pycdr2 import IdlStruct
from pycdr2.types import int8, int64, float64, uint64
from typing import List

@dataclass
class ParameterValue(IdlStruct):
    type: int8 = 0
    bool_value: bool = False
    integer_value: int64 = 0
    double_value: float64 = 0.0
    string_value: str = ""
    byte_array_value: List[int8] = field(default_factory=list)
    bool_array_value: List[bool] = field(default_factory=list)
    integer_array_value: List[int64] = field(default_factory=list)
    double_array_value: List[float64] = field(default_factory=list)
    string_array_value: List[str] = field(default_factory=list)


@dataclass
class Parameter(IdlStruct):
   name: str
   value: ParameterValue


@dataclass
class SetParametersRequest(IdlStruct):
   parameters: List[Parameter]


@dataclass
class SetParameterResult(IdlStruct):
   successful: bool
   reason: str

@dataclass
class SetParametersResponse(IdlStruct):
   results: List[SetParameterResult]


@dataclass
class ListParametersRequest(IdlStruct):
   prefixes: List[str]
   depth: uint64

@dataclass
class ListParametersResponse(IdlStruct):
   names: List[str]
   prefixes: List[str]


@dataclass
class NodeNamesRequest(IdlStruct):
   pass

@dataclass
class NodeNamesResponse(IdlStruct):
   names: List[str]


@dataclass
class NodeParamListRequest(IdlStruct):
   node_name: str


@dataclass
class ParameterInfo(IdlStruct):
   param_name: str
   param_type: str
   param_value: str


@dataclass
class NodeParamListResponse(IdlStruct):
   parameters_info: List[ParameterInfo]




SESSION_CONF = "/home/user/projects/ros-parameters-qt-config/qt_configurator/config/qt_configurator_session.json5"

class ZenohOperator:
    def __init__(self) -> None:
      self.session = zenoh.open(zenoh.config.Config.from_file(SESSION_CONF))
      # TODO: handle node names with namespace prefix
    
    def get_node_names(self):
        req = NodeNamesRequest().serialize()
        replies = self.session.get("get_node_names", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = NodeNamesResponse.deserialize(reply.ok.payload)
                # print(f">> Received {message}")
                return message.names
            except:
                print(">> Received ERROR")
                return None
    
    def get_node_custom_parameter_list(self, node_name):
        req = NodeParamListRequest(node_name=node_name).serialize()
        replies = self.session.get("get_node_parameter_list", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = NodeParamListResponse.deserialize(reply.ok.payload)
                # print(f">> Received {message}")
                return message.parameters_info
            except:
                print(">> Received ERROR")
                return None

   
    def get_node_parameters_list(self, node_name):
        req = ListParametersRequest(prefixes=[],depth=0).serialize()
        replies = self.session.get(f"{node_name[1:]}/list_parameters", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = ListParametersResponse.deserialize(reply.ok.payload)
                # print(f">> Received {message}")
                return message.names
            except:
                print(">> Received ERROR")
                return None

    # def create_parameter_value_object(self, param_type, param_value):
    #     pass

    # TODO: change ParameterInfo custom interface type to be uint8
    # TODO: move type number -> type name mapping from robot side to home side.
    # TODO: Handle a database of node parameters with types and values (use data as cache and add refreash button),
            # then you would  use the type here below
    def send_set_parameter_req(self, node_name, param_name, param_type, param_value):

        req = SetParametersRequest(parameters=[Parameter(name=param_name,value=ParameterValue(type=2, integer_value=12))]).serialize()
        replies = self.session.get(f"{node_name[1:]}/set_parameters", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = SetParametersResponse.deserialize(reply.ok.payload)
                print(f">> Received {message}")
            except:
                print(">> Received ERROR")