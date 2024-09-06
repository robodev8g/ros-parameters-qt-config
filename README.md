# ros-parameters-qt-config
Using this app, you would configure your ROS system node parameters.


## Set Node Parameter
Using ros2 cli:
``` bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 param set /calculator_node a 9

>>> Set parameter successful
```


Using service call:
``` bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 service call /calculator_node/set_parameters rcl_interfaces/srv/SetParameters "{ parameters: [{ name: b, value: {type: 2, bool_value: False, integer_value: 16, double_value: 0.0, string_value: '', byte_array_value: [], bool_array_value: [], integer_array_value: [], double_array_value: [], string_array_value: []}}] }"

>>> requester: making request: rcl_interfaces.srv.SetParameters_Request(parameters=[rcl_interfaces.msg.Parameter(name='b', value=rcl_interfaces.msg.ParameterValue(type=2, bool_value=False, integer_value=14, double_value=0.0, string_value='', byte_array_value=[], bool_array_value=[], integer_array_value=[], double_array_value=[], string_array_value=[]))])

response:
rcl_interfaces.srv.SetParameters_Response(results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='')])
```

Or you can do it shorter (Only specify the type and the coresponding value):
``` bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 service call /calculator_node/set_parameters rcl_interfaces/srv/SetParameters "{ parameters: [{ name: b, value: {type: 2, integer_value: 10}}] }"

>>> requester: making request: rcl_interfaces.srv.SetParameters_Request(parameters=[rcl_interfaces.msg.Parameter(name='b', value=rcl_interfaces.msg.ParameterValue(type=2, bool_value=False, integer_value=10, double_value=0.0, string_value='', byte_array_value=[], bool_array_value=[], integer_array_value=[], double_array_value=[], string_array_value=[]))])

response:
rcl_interfaces.srv.SetParameters_Response(results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='')])
```

### Using zenoh
The short request format can be used with zenoh as well:
``` python
req = SetParametersRequest(parameters=[Parameter(name="a",value=ParameterValue(type=2, integer_value=12))]).serialize()
replies = self.session.get("calculator_node/set_parameters", handler=zenoh.Queue(), value=req)
```
