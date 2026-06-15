#!/usr/bin/env python3
# ============================================================
#  inspect_values.py — ดูค่าดิบต่อช่องของ AMR-MNS16B-V1
#  เพื่อตัดสินใจว่าทำ weighted centroid จาก block ไหนได้
#  *** pymodbus 2.5.3 ***
#  python3 inspect_values.py --port /dev/ttyUSB0 --baud 115200 --slave 1
#
#  วิธีดู: เลื่อนแม่เหล็กผ่านเซนเซอร์ "ช้า ๆ"
#    - ค่าควรไล่ขึ้น-ลงนุ่ม ๆ ตามตำแหน่ง = graded (ทำ weighted ได้)
#    - ถ้าเด้ง 0<->max ทันที = binary (ทำ weighted ไม่ช่วย)
#  ดูด้วยว่า baseline (ตอนไม่มีแม่เหล็ก) ของแต่ละช่องเป็นเท่าไหร่
# ============================================================
import argparse, time
from pymodbus.client.sync import ModbusSerialClient

REG_DETECT = 0x20   # 8 regs, byte-packed: low=ช่องคี่ high=ช่องคู่
REG_UNIFORM = 0x10  # 16 regs, ค่าสนาม analog ต่อช่อง


def read_detect(c, slave):
    rr = c.read_holding_registers(address=REG_DETECT, count=8, unit=slave)
    if rr is None or rr.isError():
        return None
    vals = []
    for reg in rr.registers:
        vals.append(reg & 0xFF)          # ช่องคี่
        vals.append((reg >> 8) & 0xFF)   # ช่องคู่
    return vals[:16]


def read_uniform(c, slave):
    rr = c.read_holding_registers(address=REG_UNIFORM, count=16, unit=slave)
    if rr is None or rr.isError():
        return None
    # แสดงทั้งแบบ unsigned และ signed (int16) เพื่อดูว่ามีเครื่องหมายไหม
    out = []
    for reg in rr.registers:
        u = reg & 0xFFFF
        s = u - 0x10000 if u >= 0x8000 else u
        out.append((u, s))
    return out


def bar(v, vmax=255, width=10):
    n = int(width * min(v, vmax) / vmax) if vmax else 0
    return "#" * n + "-" * (width - n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/magnetic_sensor")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--block", choices=["detect", "uniform", "both"], default="both")
    args = ap.parse_args()

    c = ModbusSerialClient(method="rtu", port=args.port, baudrate=args.baud,
                           parity="N", stopbits=1, bytesize=8, timeout=0.1)
    if not c.connect():
        print(f"[ERR] เปิด {args.port} ไม่ได้")
        return
    print(f"[OK] {args.port}@{args.baud} slave={args.slave}  (Ctrl-C หยุด)\n")

    try:
        while True:
            print("\033[2J\033[H", end="")  # clear screen
            if args.block in ("detect", "both"):
                d = read_detect(c, args.slave)
                print("== 0x20-0x27 detection value (byte 0-255) ==")
                if d:
                    for i, v in enumerate(d):
                        print(f"  S{i+1:<2} {v:3d} |{bar(v)}|")
                else:
                    print("  read error")
                print()
            if args.block in ("uniform", "both"):
                u = read_uniform(c, args.slave)
                print("== 0x10-0x1F uniform field (uint / int16) ==")
                if u:
                    for i, (uv, sv) in enumerate(u):
                        print(f"  S{i+1:<2} u={uv:5d} s={sv:6d}")
                else:
                    print("  read error")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        c.close()


if __name__ == "__main__":
    main()