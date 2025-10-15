#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class CmdVelToMotorCmd(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor_cmd')

        # ── 파라미터(런치에서 덮어쓰기 가능) ──
        # '직진/후진' 판단용 선속도 임계값(절댓값) [m/s]
        self.declare_parameter('lin_thresh', 0.05)
        # '제자리 회전' 판단용 각속도 임계값(절댓값) [rad/s]
        self.declare_parameter('ang_thresh', 0.10)
        # 마지막 /cmd_vel 수신 후 N초 지나면 자동 정지
        self.declare_parameter('deadman_seconds', 1.0)

        self.lin_th = float(self.get_parameter('lin_thresh').value)
        self.ang_th = float(self.get_parameter('ang_thresh').value)
        self.deadman = float(self.get_parameter('deadman_seconds').value)

        # 퍼블리셔(/motor_cmd)와 서브스크립션(/cmd_vel) 준비
        self.pub = self.create_publisher(Float32MultiArray, '/motor_cmd', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb_twist, 10)

        # deadman 체크용 타임스탬프
        self.last_stamp = self.get_clock().now()
        # 10 Hz 주기로 deadman 감시(입력 끊기면 STOP 한 번 더 내려줘서 안전)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info('cmd_vel_to_motor_cmd is up: lin>%.2f => F/B, ang>%.2f & lin<%.2f => L/R'
                               % (self.lin_th, self.ang_th, self.lin_th))

    def _send(self, left: float, right: float):
        """
        /motor_cmd 발행 헬퍼.
        - data = [left, right, 0.0], 각각 범위는 [-1.0, 1.0]
        - 여기서는 상태만 표현하려고 ±1, 0 값만 쓴다.
          (실제 전압은 MCU에서 고정형으로 만들기 때문)
        """
        m = Float32MultiArray()
        # Saturation(안전): 혹시라도 범위 이탈 방지
        l = max(-1.0, min(1.0, left))
        r = max(-1.0, min(1.0, right))
        m.data = [l, r, 0.0]
        self.pub.publish(m)

    def cb_twist(self, msg: Twist):
        """
        /cmd_vel 수신 콜백.
        - 제자리 회전(좌/우) 판단을 선호: ang이 충분히 크고 lin이 작으면 회전.
        - 그 외엔 직진/후진/정지로 간단 매핑.
        """
        self.last_stamp = self.get_clock().now()
        lx = msg.linear.x
        az = msg.angular.z

        # 1) 제자리 회전 우선
        if abs(az) > self.ang_th and abs(lx) < self.lin_th:
            if az > 0.0:
                # 좌회전: 왼쪽 +1(전진), 오른쪽 -1(후진)
                self._send(-1.0, +1.0)
            else:
                # 우회전: 왼쪽 -1(후진), 오른쪽 +1(전진)
                self._send(+1.0, -1.0)
            return

        # 2) 직진 / 후진 / 정지
        if lx > self.lin_th:
            # 전진: 양쪽 모두 +1
            self._send(+1.0, +1.0)
        elif lx < -self.lin_th:
            # 후진: 양쪽 모두 -1
            self._send(-1.0, -1.0)
        else:
            # 정지
            self._send(0.0, 0.0)

    def watchdog(self):
        """
        deadman: 마지막 /cmd_vel 수신 후 deadman_seconds가 지났으면 정지.
        (teleop 창 포커스 잃거나 입력 끊김 대비)
        """
        dt = (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
        if dt > self.deadman:
            self._send(0.0, 0.0)

def main():
    rclpy.init()
    rclpy.spin(CmdVelToMotorCmd())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
