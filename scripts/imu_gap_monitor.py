#!/usr/bin/env python3
"""Domain-77 probe: wall-clock receive gaps on /imu/data_ekf, ESKF readiness and gate transitions."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray

class M(Node):
    def __init__(self):
        super().__init__("imu_gap_monitor")
        self.last = None; self.maxgap = 0.0; self.n = 0; self.t0 = time.time()
        self.ready = None; self.gate = None
        self.create_subscription(Imu, "/imu/data_ekf", self.on_imu, qos_profile_sensor_data)
        q = QoSProfile(depth=1); q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, "/ekf/heading_verified", self.on_gate, q)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        self.create_timer(1.0, self.tick)
    def log(self, s): print(f"{time.time():.3f} {time.strftime('%H:%M:%S')} {s}", flush=True)
    def on_imu(self, m):
        now = time.time()
        if self.last is not None:
            g = now - self.last
            self.maxgap = max(self.maxgap, g)
            if g > 0.05: self.log(f"IMU RECEIVE GAP {g*1000:.0f} ms")
        self.last = now; self.n += 1
    def on_gate(self, m):
        if m.data != self.gate: self.log(f"/ekf/heading_verified -> {m.data}")
        self.gate = m.data
    def on_diag(self, m):
        for st in m.status:
            if "eskf" not in st.name.lower() and "eskf" not in st.hardware_id.lower(): continue
            kv = {k.key: k.value for k in st.values}
            r = kv.get("imu_readiness")
            if r != self.ready: self.log(f"imu_readiness -> {r} (recv_gap={kv.get('imu_last_receive_gap_sec')} max_stamp_gap={kv.get('imu_max_stamp_gap_sec')} recoveries={kv.get('imu_gap_recoveries')} resets={kv.get('imu_gap_resets')} observed={kv.get('heading_observed')})")
            self.ready = r
    def tick(self):
        self.log(f"imu {self.n} Hz, max gap 1s {self.maxgap*1000:.0f} ms, readiness={self.ready} gate={self.gate}")
        self.n = 0; self.maxgap = 0.0

rclpy.init(); n = M()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
