#!/usr/bin/env python3
# ============================================================
#  line_following_node.py
#  ROS 2 (Jazzy) line following — อ่าน AMR-MNS16B-V1 (Modbus RTU)
#  แล้วคุมหุ่น diff drive ตามแถบแม่เหล็ก publish /cmd_vel_line
#
#  รวมตัวอ่านเซนเซอร์ + ตัวควบคุมไว้ใน node เดียว (ไม่ต้องมี odom)
#  ลูป: อ่าน 16 ช่อง -> centroid -> PD steering -> Twist
#
#  *** pymodbus 2.5.3 เท่านั้น ***
#    pip install --break-system-packages "pymodbus==2.5.3" pyserial
#
#  อ้างอิง datasheet: FC03H, register 0x28 = 16 digital outputs (bitmask)
#    bit0=S1 .. bit15=S16 ; baud 115200, 8N1, slave 1
# ============================================================
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pymodbus.client.sync import ModbusSerialClient   # pymodbus 2.5.3

REG_DIGITAL_OUTPUTS = 0x28


class LineFollowingNode(Node):
    def __init__(self):
        super().__init__("line_following")

        # ---- เซนเซอร์ (Modbus) ----
        self.declare_parameter("port", "/dev/magnetic_sensor")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("num_channels", 16)
        self.declare_parameter("msb_first", False)     # True ถ้า bit15=S1
        self.declare_parameter("invert", False)        # True ถ้า register 0=เจอเส้น

        # ---- การควบคุม ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_line")
        self.declare_parameter("forward_speed", 0.1)  # m/s
        self.declare_parameter("kp", 0.6)
        self.declare_parameter("kd", 0.2)
        self.declare_parameter("position_deadband", 0.05)
        self.declare_parameter("steer_sign", -1.0)      # พลิก -1.0 ถ้าแก้ผิดทาง
        self.declare_parameter("max_angular", 1.2)     # rad/s
        self.declare_parameter("slowdown_gain", 0.8)   # ชะลอตอน error มาก (0=ไม่ชะลอ)
        self.declare_parameter("min_forward_speed", 0.05)
        self.declare_parameter("control_rate", 50.0)   # Hz (= อัตราอ่านเซนเซอร์)

        # ---- พฤติกรรมตอนเส้นหาย ----
        self.declare_parameter("on_lost", "stop")      # "stop" | "search"
        self.declare_parameter("search_angular", 0.4)  # rad/s ตอน search

        g = self.get_parameter
        self.port = g("port").value
        self.baud = g("baudrate").value
        self.slave_id = g("slave_id").value
        self.n_ch = g("num_channels").value
        self.msb_first = g("msb_first").value
        self.invert = g("invert").value

        self.forward = float(g("forward_speed").value)
        self.kp = float(g("kp").value)
        self.kd = float(g("kd").value)
        self.deadband = float(g("position_deadband").value)
        self.steer_sign = float(g("steer_sign").value)
        self.max_ang = float(g("max_angular").value)
        self.slow_gain = float(g("slowdown_gain").value)
        self.min_fwd = float(g("min_forward_speed").value)
        self.on_lost = g("on_lost").value
        self.search_ang = float(g("search_angular").value)
        rate = float(g("control_rate").value)

        # ---- state ----
        self.client = None
        self.prev_error = 0.0
        self.last_position = 0.0
        self._active = False
        self.dt = 1.0 / rate

        self.pub = self.create_publisher(Twist, g("cmd_vel_topic").value, 10)
        self._connect()
        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f"line_following: {self.port}@{self.baud} slave={self.slave_id} "
            f"forward={self.forward} kp={self.kp} -> {g('cmd_vel_topic').value}"
        )

    # --------------------------------------------------------
    def _connect(self):
        self.client = ModbusSerialClient(
            method="rtu", port=self.port, baudrate=self.baud,
            parity="N", stopbits=1, bytesize=8, timeout=0.05,
        )
        if self.client.connect():
            self.get_logger().info(f"sensor connected: {self.port}")
            return True
        self.get_logger().warn(f"cannot open {self.port} — จะลองใหม่")
        return False

    # --------------------------------------------------------
    def _read_channels(self):
        """อ่าน register 0x28 -> list 0/1 ยาว num_channels (None ถ้าอ่านพลาด)"""
        try:
            rr = self.client.read_holding_registers(
                address=REG_DIGITAL_OUTPUTS, count=1, unit=self.slave_id
            )
        except Exception as e:
            self.get_logger().warn(f"modbus read error: {e}")
            return None
        if rr is None or rr.isError():
            return None
        word = rr.registers[0] & 0xFFFF
        bits = [(word >> i) & 1 for i in range(self.n_ch)]   # bit0 = S1
        if self.msb_first:
            bits = bits[::-1]
        if self.invert:
            bits = [0 if b else 1 for b in bits]
        return bits

    # --------------------------------------------------------
    def _compute_position(self, ch):
        """centroid ช่องที่เจอเส้น -> -1.0(ซ้าย S1) .. +1.0(ขวา S16); None ถ้าไม่เจอ"""
        active = [i for i, v in enumerate(ch) if v]
        if not active:
            return None
        centroid = sum(active) / len(active)
        center = (self.n_ch - 1) / 2.0
        return (centroid - center) / center

    # --------------------------------------------------------
    def _publish(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub.publish(m)

    def _stop(self):
        self._publish(0.0, 0.0)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # --------------------------------------------------------
    def _on_timer(self):
        # เซนเซอร์หลุด -> ลองต่อใหม่ + หยุด
        if self.client is None or not self.client.is_socket_open():
            self._connect()
            self._stop()
            self._active = False
            return

        ch = self._read_channels()
        if ch is None:           # อ่านพลาดรอบนี้ -> หยุดไว้ก่อน (ปลอดภัย)
            self._stop()
            self._active = False
            return

        pos = self._compute_position(ch)

        # เส้นหาย
        if pos is None:
            if self.on_lost == "search":
                direction = math.copysign(1.0, -self.prev_error) if self.prev_error else 1.0
                self._publish(0.0, self.steer_sign * self.search_ang * direction)
            else:
                self._stop()
            self._active = False
            return

        self.last_position = pos

        # deadband -> ถือว่าอยู่กลาง
        p = 0.0 if abs(pos) <= self.deadband else pos

        # ---- PD ----
        error = 0.0 - p
        if not self._active:     # เพิ่งกลับมาเจอเส้น: กันเทอม D กระชาก
            d_err = 0.0
            self._active = True
        else:
            d_err = (error - self.prev_error) / self.dt
        self.prev_error = error
        u = self.kp * error + self.kd * d_err

        # เดินหน้า + ชะลอเมื่อ error มาก
        vx = self.forward * (1.0 - self.slow_gain * abs(p))
        vx = self._clamp(vx, self.min_fwd, self.forward)
        wz = self._clamp(self.steer_sign * u, -self.max_ang, self.max_ang)
        self._publish(vx, wz)

    # --------------------------------------------------------
    def destroy_node(self):
        self._stop()
        if self.client is not None:
            self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()