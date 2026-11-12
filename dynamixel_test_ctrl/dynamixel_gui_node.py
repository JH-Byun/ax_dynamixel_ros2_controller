import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from tkinter import Tk, Button, Label, StringVar
from functools import partial

class DynamixelGuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.declare_parameter('ids', [4, 7])
        self.ids = self.get_parameter('ids').get_parameter_value().integer_array_value

        self.current_pos = {id: 0 for id in self.ids}
        self.current_vel = {id: 0 for id in self.ids}
        self.current_load = {id: 0 for id in self.ids}
        self.current_modes = {id: 'position' for id in self.ids}
        self.goal_values = {id: 0 for id in self.ids}
        self.pos_initialized = {dxl_id: False for dxl_id in self.ids}


        self.label_vars = {}

        self.pubs = {}
        for dxl_id in self.ids:
            self.create_subscription(String, f'/dxl{dxl_id}/state/mode', lambda msg, i=dxl_id: self.update_mode(i, msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/state/position', lambda msg, i=dxl_id: self.update_value(i, 'pos', msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/state/velocity', lambda msg, i=dxl_id: self.update_value(i, 'vel', msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/state/load', lambda msg, i=dxl_id: self.update_value(i, 'load', msg.data), 10)

            self.pubs[dxl_id] = {
                'goal_position': self.create_publisher(Int32, f'/dxl{dxl_id}/goal_position', 10),
                'goal_velocity': self.create_publisher(Int32, f'/dxl{dxl_id}/goal_velocity', 10),
                'set_mode': self.create_publisher(String, f'/dxl{dxl_id}/set_mode', 10),
            }

        self.root = Tk()
        self.root.title("Dynamixel GUI")


        for idx, dxl_id in enumerate(self.ids):
            Label(self.root, text=f"ID {dxl_id}").grid(row=idx, column=0)

            Button(self.root, text="Position", command=partial(self.change_mode, dxl_id, 'position')).grid(row=idx, column=1)
            Button(self.root, text="Velocity", command=partial(self.change_mode, dxl_id, 'velocity')).grid(row=idx, column=2)
            Label(self.root, text="AX-18A").grid(row=idx, column=3)

            Button(self.root, text="+", command=partial(self.increment, dxl_id)).grid(row=idx, column=4)
            Button(self.root, text="-", command=partial(self.decrement, dxl_id)).grid(row=idx, column=5)

            self.label_vars[dxl_id] = StringVar()
            self.label_vars[dxl_id].set(self.format_label(dxl_id))
            Label(self.root, textvariable=self.label_vars[dxl_id], width=60, anchor='w').grid(row=idx, column=6, columnspan=3)

        self.get_logger().info("Starting Dynamixel GUI Node")
        self.root.after(100, self.tk_loop)
        self.root.mainloop()

    def format_label(self, dxl_id):
        mode = self.current_modes[dxl_id]
        pos = self.current_pos[dxl_id]
        vel = self.current_vel[dxl_id]
        load = self.current_load[dxl_id]
        goal = self.goal_values[dxl_id]
        return f"[Mode: {mode}] Pos: {pos} | Vel: {vel} | Load: {load} | Goal: {goal}"

    def update_label(self, dxl_id):
        self.label_vars[dxl_id].set(self.format_label(dxl_id))

    def change_mode(self, dxl_id, mode):
        if not self.pos_initialized[dxl_id]:
            return
        self.current_modes[dxl_id] = mode
        if mode == 'position':
            self.goal_values[dxl_id] = self.current_pos[dxl_id]
        else:
            self.goal_values[dxl_id] = 0
        self.update_label(dxl_id)
        self.pubs[dxl_id]['set_mode'].publish(String(data=mode))

    def update_mode(self, dxl_id, mode):
        if self.current_modes[dxl_id] != mode:
            self.goal_values[dxl_id] = self.current_pos[dxl_id] if mode == 'position' else 0
        self.current_modes[dxl_id] = mode
        self.update_label(dxl_id)

    def update_value(self, dxl_id, key, value):
        if key == 'pos':
            self.current_pos[dxl_id] = value
            if not self.pos_initialized[dxl_id]:
                self.goal_values[dxl_id] = value
            self.pos_initialized[dxl_id] = True
        elif key == 'vel':
            self.current_vel[dxl_id] = value
        elif key == 'load':
            self.current_load[dxl_id] = value
        self.update_label(dxl_id)

    def increment(self, dxl_id):
        if not self.pos_initialized[dxl_id]:
            return
        mode = self.current_modes[dxl_id]
        step = {'position': 10, 'velocity': 20}[mode]
        max_val = {'position': 1023, 'velocity': 100}[mode]
        self.goal_values[dxl_id] = min(self.goal_values[dxl_id] + step, max_val)
        self.update_label(dxl_id)
        self.publish_goal(dxl_id)

    def decrement(self, dxl_id):
        if not self.pos_initialized[dxl_id]:
            return
        mode = self.current_modes[dxl_id]
        step = {'position': 10, 'velocity': 20}[mode]
        min_val = {'position': 0, 'velocity': -100}[mode]
        self.goal_values[dxl_id] = max(self.goal_values[dxl_id] - step, min_val)
        self.update_label(dxl_id)
        self.publish_goal(dxl_id)

    def publish_goal(self, dxl_id):
        mode = self.current_modes[dxl_id]
        value = self.goal_values[dxl_id]
        if mode == 'position':
            self.pubs[dxl_id]['goal_position'].publish(Int32(data=value))
        elif mode == 'velocity':
            self.pubs[dxl_id]['goal_velocity'].publish(Int32(data=value))

    def tk_loop(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.update()
        self.root.after(50, self.tk_loop)

def main(args=None):
    rclpy.init(args=args)
    DynamixelGuiNode()
    rclpy.shutdown()
