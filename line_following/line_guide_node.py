#!/usr/bin/env python3
# ============================================================
#  line_guide_node.py
#  ROS 2 (Jazzy) อ่าน AMR-MNS16B-V1 (Modbus RTU, RS-485)
#  คำนวณตำแหน่งเส้นแบบ "weighted centroid" จาก register 0x20-0x27
#  (detection value ต่อช่อง byte 0-255) -> position ลื่น ไม่กระโดดเป็นขั้น
#
#  *** pymodbus 2.5.3 ***
#
#  Publishes:
#    ~/line_position  std_msgs/Float32  (-1.0 ซ้าย S1 .. +1.0 ขวา S16)
#    ~/line_detected  std_msgs/Bool
#    ~/channels       std_msgs/Int32MultiArray  (ค่า 16 ช่อง 0-255 หลังลบ baseline)
#
#  register: FC03H, 0x20-0x27 = 8 regs, byte-packed
#    low byte  = ช่องคี่ (S1,S3,..)   high byte = ช่องคู่ (S2,S4,..)
# ============================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32MultiArray

from pymodbus.client.sync import ModbusSerialClient   # pymodbus 2.5.3

REG_DETECTION_BASE = 0x20   # 8 regs (16 ช่อง)


class LineGuideNode(Node):
    def __init__(self):
        super().__init__("line_guide_node")

        self.declare_parameter("port", "/dev/magnetic_sensor")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("num_channels", 16)
        self.declare_parameter("reverse", False)          # True ถ้าซ้าย/ขวากลับด้าน (S1 อยู่ขวา)

        # --- weighted centroid ---
        self.declare_parameter("threshold", 5)            # ตัด noise: ค่าต่ำกว่านี้ = 0
        self.declare_parameter("min_total", 15)           # ผลรวมน้ำหนักต่ำกว่านี้ = เส้นหาย
        self.declare_parameter("poll_rate", 50.0)         # Hz
        self.declare_parameter("hold_last_on_lost", True)

        g = self.get_parameter
        self.port = g("port").value
        self.baud = g("baudrate").value
        self.slave_id = g("slave_id").value
        self.n_ch = g("num_channels").value
        self.reverse = g("reverse").value
        self.threshold = int(g("threshold").value)
        self.min_total = int(g("min_total").value)
        self.hold_last = g("hold_last_on_lost").value
        rate = float(g("poll_rate").value)

        self.pub_position = self.create_publisher(Float32, "~/line_position", 10)
        self.pub_detected = self.create_publisher(Bool, "~/line_detected", 10)
        self.pub_channels = self.create_publisher(Int32MultiArray, "~/channels", 10)

        self.client = None
        self.last_position = 0.0
        self._connect()

        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"line_guide_node (weighted): {self.port}@{self.baud} slave={self.slave_id} "
            f"thr={self.threshold} min_total={self.min_total}"
        )

    # --------------------------------------------------------
    def _connect(self):
        self.client = ModbusSerialClient(
            method="rtu", port=self.port, baudrate=self.baud,
            parity="N", stopbits=1, bytesize=8, timeout=0.05,
        )
        if self.client.connect():
            self.get_logger().info(f"connected to {self.port}")
            return True
        self.get_logger().warn(f"cannot open {self.port} — จะลองใหม่")
        return False

    # --------------------------------------------------------
    def _read_values(self):
        """อ่าน 0x20-0x27 -> list ค่า 0-255 ยาว 16 (None ถ้าพลาด)"""
        nregs = (self.n_ch + 1) // 2
        try:
            rr = self.client.read_holding_registers(
                address=REG_DETECTION_BASE, count=nregs, unit=self.slave_id
            )
        except Exception as e:
            self.get_logger().warn(f"modbus read error: {e}")
            return None
        if rr is None or rr.isError():
            return None
        vals = []
        for reg in rr.registers:
            vals.append(reg & 0xFF)          # ช่องคี่
            vals.append((reg >> 8) & 0xFF)   # ช่องคู่
        vals = vals[:self.n_ch]
        if self.reverse:
            vals = vals[::-1]
        return vals

    # --------------------------------------------------------
    def _weighted_position(self, vals):
        """weighted centroid -> (position -1..+1, total) ; position=None ถ้าเส้นหาย"""
        w = [v if v >= self.threshold else 0 for v in vals]   # ตัด noise
        total = sum(w)
        if total < self.min_total:
            return None, total
        centroid = sum(i * wi for i, wi in enumerate(w)) / total
        center = (self.n_ch - 1) / 2.0
        return (centroid - center) / center, total

    # --------------------------------------------------------
    def _on_timer(self):
        if self.client is None or not self.client.is_socket_open():
            self._connect()
            return

        vals = self._read_values()
        if vals is None:
            return

        w = [v if v >= self.threshold else 0 for v in vals]
        self.pub_channels.publish(Int32MultiArray(data=[int(x) for x in w]))

        pos, _ = self._weighted_position(vals)
        detected = pos is not None
        if pos is None:
            pos = self.last_position if self.hold_last else 0.0
        else:
            self.last_position = pos

        self.pub_position.publish(Float32(data=float(pos)))
        self.pub_detected.publish(Bool(data=detected))

    # --------------------------------------------------------
    def destroy_node(self):
        if self.client is not None:
            self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineGuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()