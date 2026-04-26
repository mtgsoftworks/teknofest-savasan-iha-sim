#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamSetV2

rclpy.init()
node = Node("param_setter")
cli = node.create_client(ParamSetV2, "/mavros/param/set")
cli.wait_for_service(timeout_sec=5)

params = [
    # Keep Gazebo airspeed bridge enabled and use the model-provided airspeed sensor.
    ("SIM_GZ_EN_ASPD", 1, 2),
    ("SENS_EN_ARSPDSIM", 0, 2),
    ("NAV_DLL_ACT", 0, 2),
    ("COM_PREARM_MODE", 2, 2),
]

for pname, pval, ptype in params:
    req = ParamSetV2.Request()
    req.force_set = True
    req.param_id = pname
    req.value.type = ptype
    if ptype == 2:
        req.value.integer_value = int(pval)
        req.value.double_value = 0.0
    else:
        req.value.integer_value = 0
        req.value.double_value = float(pval)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    result = future.result()
    if result is None:
        print(f"{pname} set: TIMEOUT")
    else:
        print(f"{pname} set: success={result.success}")

node.destroy_node()
rclpy.shutdown()
