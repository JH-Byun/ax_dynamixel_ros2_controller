import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from std_msgs.msg import Int32, Bool, String
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

ADDR_CW_LIMIT = 6
ADDR_CCW_LIMIT = 8
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'


def decode_direction(value):
    magnitude = value & 1023
    return -magnitude if value & 1024 else magnitude


class DynamixelMasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.declare_parameter('ids', [4, 7])
        self.declare_parameter(
            'position_speed', 100,
            ParameterDescriptor(
                description='AX-18A joint moving speed in raw units (1–1023).',
                read_only=True,
                integer_range=[IntegerRange(from_value=1, to_value=1023, step=1)],
            ),
        )
        self.position_speed = self.get_parameter('position_speed').value
        if not 1 <= self.position_speed <= 1023:
            raise ValueError('position_speed must be between 1 and 1023')
        self.dxl_ids = list(self.get_parameter('ids').value)
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        if not self.portHandler.openPort():
            raise RuntimeError(f'Cannot open {DEVICENAME}')
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.portHandler.closePort()
            raise RuntimeError(f'Cannot set baud rate to {BAUDRATE}')
        self.current_mode = {}
        self.joint_limits = {}
        self.pub = {}
        for dxl_id in self.dxl_ids:
            self.pub[dxl_id] = {
                key: self.create_publisher(msg_type, f'/dxl{dxl_id}/state/{topic}', 10)
                for key, msg_type, topic in (
                    ('position', Int32, 'position'), ('velocity', Int32, 'velocity'),
                    ('load', Int32, 'load'), ('mode', String, 'mode'),
                    ('torque', Bool, 'torque_enabled'))
            }
            model, result, error = self.packetHandler.ping(self.portHandler, dxl_id)
            if not self.check_result(dxl_id, 'ping', result, error):
                continue
            if model != 18:
                self.get_logger().error(f'[ID {dxl_id}] Expected AX-18A model 18, got {model}; skipping')
                continue
            limits, result, error = self.packetHandler.readTxRx(self.portHandler, dxl_id, ADDR_CW_LIMIT, 4)
            if not self.check_result(dxl_id, 'read angle limits', result, error):
                continue
            cw = limits[0] | limits[1] << 8
            ccw = limits[2] | limits[3] << 8
            self.joint_limits[dxl_id] = (cw, ccw) if ccw else (0, 1023)
            self.current_mode[dxl_id] = None
            self.set_operating_mode(dxl_id, 'position')
            self.create_subscription(Int32, f'/dxl{dxl_id}/goal_position', lambda msg, i=dxl_id: self.send_command(i, 'position', msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/goal_velocity', lambda msg, i=dxl_id: self.send_command(i, 'velocity', msg.data), 10)
            self.create_subscription(String, f'/dxl{dxl_id}/set_mode', lambda msg, i=dxl_id: self.set_operating_mode(i, msg.data), 10)
        self.timer = self.create_timer(0.05, self.read_states)
        self.get_logger().info(f'AX-18A controller: {DEVICENAME}, {BAUDRATE} bps, IDs {self.dxl_ids}')

    def check_result(self, dxl_id, action, result, error):
        if result != COMM_SUCCESS or error:
            detail = (self.packetHandler.getTxRxResult(result) if result != COMM_SUCCESS
                      else self.packetHandler.getRxPacketError(error))
            options = {'throttle_duration_sec': 2.0} if action == 'read states' else {}
            self.get_logger().error(f'[ID {dxl_id}] {action}: {detail}', **options)
            return False
        return True

    def write(self, dxl_id, address, value, size=2):
        method = self.packetHandler.write1ByteTxRx if size == 1 else self.packetHandler.write2ByteTxRx
        result, error = method(self.portHandler, dxl_id, address, value)
        return self.check_result(dxl_id, f'write address {address}', result, error)

    def send_command(self, dxl_id, mode, value):
        if self.current_mode.get(dxl_id) != mode:
            return
        if mode == 'position':
            low, high = self.joint_limits[dxl_id]
            if not low <= value <= high:
                self.get_logger().warn(f'[ID {dxl_id}] Position must be {low}..{high}')
                return
            self.write(dxl_id, ADDR_GOAL_POSITION, value)
        elif mode == 'velocity':
            if not -1023 <= value <= 1023:
                self.get_logger().warn('Wheel speed must be -1023..1023')
                return
            self.write(dxl_id, ADDR_MOVING_SPEED, abs(value) | (1024 if value < 0 else 0))

    def set_operating_mode(self, dxl_id, mode):
        if mode not in ('position', 'velocity'):
            self.get_logger().warn('AX-18A supports position and wheel velocity modes; no current mode')
            return
        if self.current_mode.get(dxl_id) == mode:
            return
        if not self.write(dxl_id, ADDR_TORQUE_ENABLE, 0, 1):
            return
        self.current_mode[dxl_id] = None
        self.pub[dxl_id]['torque'].publish(Bool(data=False))
        cw, ccw = self.joint_limits[dxl_id] if mode == 'position' else (0, 0)
        for address, value in ((ADDR_CW_LIMIT, cw), (ADDR_CCW_LIMIT, ccw)):
            if not self.write(dxl_id, address, value):
                return
        if mode == 'position':
            position, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if not self.check_result(dxl_id, 'read initial position', result, error):
                return
            if not cw <= position <= ccw:
                self.get_logger().error(f'[ID {dxl_id}] Position outside joint limits; torque remains off')
                return
            if not self.write(dxl_id, ADDR_GOAL_POSITION, position):
                return
        # In joint mode use a limited speed; in wheel mode zero means stop.
        if not self.write(dxl_id, ADDR_MOVING_SPEED, self.position_speed if mode == 'position' else 0):
            return
        if not self.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 1):
            return
        self.current_mode[dxl_id] = mode
        self.pub[dxl_id]['mode'].publish(String(data=mode))
        self.pub[dxl_id]['torque'].publish(Bool(data=True))
        self.get_logger().info(f'[ID {dxl_id}] Mode change → {mode}')

    def read_states(self):
        # AX-18A uses ordinary Protocol 1.0 READ, not bulk read.
        for dxl_id in self.current_mode:
            data, result, error = self.packetHandler.readTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION, 6)
            if not self.check_result(dxl_id, 'read states', result, error):
                continue
            values = [data[i] | data[i + 1] << 8 for i in (0, 2, 4)]
            for key, value in zip(('position', 'velocity', 'load'),
                                  (values[0], decode_direction(values[1]), decode_direction(values[2]))):
                self.pub[dxl_id][key].publish(Int32(data=value))
            mode = self.current_mode[dxl_id]
            if mode:
                self.pub[dxl_id]['mode'].publish(String(data=mode))

    def destroy_node(self):
        for dxl_id in self.current_mode:
            self.write(dxl_id, ADDR_TORQUE_ENABLE, 0, 1)
        self.portHandler.closePort()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamixelMasterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
