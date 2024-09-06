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





SESSION_CONF = "/home/user/projects/ros-parameters-qt-config/qt_configurator/config/qt_configurator_session.json5"

class ZenohOperator:
    def __init__(self) -> None:
      self.session = zenoh.open(zenoh.config.Config.from_file(SESSION_CONF))

    def send_set_parameter_req(self):
        req = SetParametersRequest(parameters=[Parameter(name="a",value=ParameterValue(type=2, integer_value=12))]).serialize()
        replies = self.session.get("calculator_node/set_parameters", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = SetParametersResponse.deserialize(reply.ok.payload)
                print(f">> Received {message}")
            except:
                print(">> Received ERROR")
   
    def get_node_parameters_list(self):
        req = ListParametersRequest(prefixes=[],depth=0).serialize()
        replies = self.session.get("calculator_node/list_parameters", handler=zenoh.Queue(), value=req)
        for reply in replies.receiver:
            try:
                message = ListParametersResponse.deserialize(reply.ok.payload)
                print(f">> Received {message}")
            except:
                print(">> Received ERROR")
