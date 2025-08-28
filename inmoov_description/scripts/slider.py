#!/usr/bin/env python3
import math, time, threading, argparse, os, sys
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray

# Optional: load YAML if needed
try:
    import yaml
except Exception:
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

DEFAULT_CTRL = 'inmoov_position_controller'
RSP_NODE = '/robot_state_publisher'
CMD_TOPIC_TMPL = '/{ctrl}/commands'
PUBLISH_HZ = 20.0

def get_params(node: Node, service_ns: str, names):
    cli = node.create_client(GetParameters, f"{service_ns}/get_parameters")
    if not cli.wait_for_service(timeout_sec=2.0):
        return None
    req = GetParameters.Request()
    req.names = names
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    if not fut.done() or fut.result() is None:
        return None
    vals = []
    for v in fut.result().values:
        if v.type == 5:      # string array
            vals.append(list(v.string_array_value))
        elif v.type == 4:    # string
            vals.append(v.string_value)
        else:
            vals.append(None)
    return vals

def get_controller_joints(node: Node, controller: str, yaml_path: str | None):
    # Strategy 1: controller node directly
    vals = get_params(node, f'/{controller}', ['joints'])
    if vals and isinstance(vals[0], list) and vals[0]:
        node.get_logger().info(f"Found joints via /{controller}/get_parameters")
        return vals[0]

    # Strategy 2: try controller_manager with a dotted param
    vals = get_params(node, '/controller_manager', [f'{controller}.joints'])
    if vals and isinstance(vals[0], list) and vals[0]:
        node.get_logger().info("Found joints via /controller_manager/get_parameters")
        return vals[0]

    # Strategy 3: read YAML
    if not yaml_path:
        # best-guess default inside the package
        if get_package_share_directory:
            try:
                share = get_package_share_directory('inmoov_description')
                yaml_path = os.path.join(share, 'config', 'controllers.yaml')
            except Exception:
                yaml_path = None
    if yaml_path and os.path.exists(yaml_path):
        if yaml is None:
            raise RuntimeError(
                f"Could not query parameters and PyYAML is not installed.\n"
                f"Install it: sudo apt install python3-yaml\n"
                f"Or pass --yaml explicitly."
            )
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        # expect top-level "<controller>: ros__parameters: joints: [...]"
        if controller in data and 'ros__parameters' in data[controller]:
            joints = data[controller]['ros__parameters'].get('joints')
            if isinstance(joints, list) and joints:
                node.get_logger().info(f"Loaded joints from YAML: {yaml_path}")
                return joints
    return None

def parse_limits_from_urdf(urdf: str):
    limits = {}
    try:
        root = ET.fromstring(urdf)
    except Exception:
        return limits
    for j in root.findall('joint'):
        name = j.get('name')
        jtype = j.get('type')
        mimic = j.find('mimic') is not None
        lo = hi = None
        if jtype == 'continuous':
            lo, hi = -math.pi, math.pi
        else:
            lim = j.find('limit')
            if lim is not None:
                l = lim.get('lower'); u = lim.get('upper')
                if l is not None and u is not None:
                    lo, hi = float(l), float(u)
                    if lo > hi:
                        lo, hi = hi, lo
        limits[name] = (lo, hi, mimic)
    return limits

def get_robot_description(node: Node):
    vals = get_params(node, RSP_NODE, ['robot_description'])
    if vals and isinstance(vals[0], str) and vals[0]:
        return vals[0]
    return None

class SliderGUI(Node):
    def __init__(self, root: tk.Tk, controller: str, yaml_path: str | None, rate_hz: float):
        super().__init__('inmoov_slider_gui')
        self.root = root
        self.ctrl = controller
        self.cmd_topic = CMD_TOPIC_TMPL.format(ctrl=self.ctrl)
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        # Wait up to ~15s for controller to expose params; if not, read YAML
        joints = None
        deadline = time.time() + 15.0
        while rclpy.ok() and time.time() < deadline and joints is None:
            joints = get_controller_joints(self, controller, yaml_path)
            if joints:
                break
            self.get_logger().info("Waiting for controller joints (or reading YAML)...")
            time.sleep(1.0)
        if not joints:
            raise RuntimeError(
                f"Could not get joint list for '{controller}'. "
                f"Is the controller running and/or is the YAML path correct?"
            )

        urdf = get_robot_description(self) or ""
        lims = parse_limits_from_urdf(urdf)

        frm = ttk.Frame(root, padding=10)
        frm.pack(fill='both', expand=True)
        header = ttk.Label(frm, text=f"Publishing to {self.cmd_topic} @ {rate_hz:.0f} Hz")
        header.grid(column=0, row=0, columnspan=3, sticky='w')

        self.vars = []
        self.names = []
        row = 1
        for j in joints:
            self.names.append(j)
            lo, hi, mimic = lims.get(j, (None, None, False))
            if mimic:
                self.get_logger().warn(f"Joint '{j}' has <mimic>. Remove it from the controller list.")
            if lo is None or hi is None:
                lo, hi = -1.0, 1.0
            v = tk.DoubleVar(value=(lo + hi) / 2.0)
            self.vars.append(v)
            ttk.Label(frm, text=j).grid(column=0, row=row, sticky='w')
            s = ttk.Scale(frm, from_=lo, to=hi, orient='horizontal', variable=v)
            s.grid(column=1, row=row, sticky='ew', padx=8)
            val = ttk.Label(frm, text=f"{v.get():.3f}")
            val.grid(column=2, row=row, sticky='e')
            def mk_cb(var=v, lbl=val):
                def cb(_evt=None):
                    lbl.config(text=f"{var.get():.3f}")
                return cb
            s.bind("<B1-Motion>", mk_cb())
            s.bind("<ButtonRelease-1>", mk_cb())
            row += 1
        frm.columnconfigure(1, weight=1)

        self.running = True
        self.rate = 1.0 / max(1e-6, rate_hz)
        threading.Thread(target=self._publisher_loop, daemon=True).start()
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _publisher_loop(self):
        while self.running and rclpy.ok():
            msg = Float64MultiArray()
            msg.data = [v.get() for v in self.vars]
            self.pub.publish(msg)
            time.sleep(self.rate)

    def _on_close(self):
        self.running = False
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

def main():
    ap = argparse.ArgumentParser(description="InMoov sliders for ros2_control")
    ap.add_argument('--controller', default=DEFAULT_CTRL, help='Controller name (node name)')
    ap.add_argument('--yaml', default=None, help='Path to controllers.yaml (fallback)')
    ap.add_argument('--rate', type=float, default=PUBLISH_HZ, help='Publish rate (Hz)')
    args = ap.parse_args()

    # Tk on Ubuntu sometimes needs this package:
    # sudo apt install python3-tk
    rclpy.init()
    root = tk.Tk()
    root.title("InMoov Joint Sliders (ros2_control)")
    SliderGUI(root, args.controller, args.yaml, args.rate)
    root.mainloop()

if __name__ == "__main__":
    main()
