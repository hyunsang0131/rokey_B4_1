"""gear_gripper.py - RG2 그리퍼를 디지털 I/O로 여닫는 클래스.

원본 DRL의 '토글' 방식(DI1 상태를 보고 반대로 동작) 대신
grip() / release() 를 명시적으로 호출합니다. 집기 실패 시 순서가 뒤바뀌는 문제를 막습니다.
"""
import time

import rclpy

from rokey1 import gear_config as cfg


class GearError(RuntimeError):
    """공정을 멈춰야 하는 오류."""


class GearGripper:
    def __init__(self, dr, logger, node=None, strict=True):
        self.dr = dr
        self.log = logger
        self.node = node
        self.strict = strict
        self._client = None
        self._service_ok = None     # None: 아직 확인 안 함
        if node is not None and cfg.USE_GRIPPER_SERVICE:
            from onrobot_rg_msgs.srv import SetCommand
            self._SetCommand = SetCommand
            self._client = node.create_client(SetCommand, cfg.GRIPPER_SERVICE)

    def _send_service(self, cmd):
        """OnRobot 서비스 호출('c' 닫기 / 'o' 열기). 없으면 처음 한 번만 경고하고 건너뜀."""
        if self._client is None or self._service_ok is False:
            return
        if self._service_ok is None:
            self._service_ok = self._client.wait_for_service(timeout_sec=2.0)
            if not self._service_ok:
                self.log.warn("%s 서비스가 없어 디지털 I/O로만 그리퍼를 제어합니다." % cfg.GRIPPER_SERVICE)
                return
        req = self._SetCommand.Request()
        req.command = cmd
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        res = future.result()
        if res is None or not res.success:
            self.log.warn("그리퍼 서비스 '%s' 실패: %s" % (cmd, res.message if res else "timeout"))

    def is_holding(self):
        return self.dr.get_digital_input(cfg.DI_GRIP) == 1

    def release(self):
        """열기: DO1=OFF, DO2=ON."""
        self.dr.set_digital_output(cfg.DO_CLOSE, 0)
        self.dr.set_digital_output(cfg.DO_OPEN, 1)
        self._send_service("o")
        time.sleep(cfg.RELEASE_WAIT)
        if self.is_holding():
            self.log.warn("그리퍼를 열었는데 DI%d(잡힘 감지)가 아직 ON 입니다." % cfg.DI_GRIP)

    def grip(self):
        """닫기: DO1=ON, DO2=OFF. 잡힘 감지(DI1)가 들어올 때까지 기다립니다."""
        self.dr.set_digital_output(cfg.DO_OPEN, 0)
        self.dr.set_digital_output(cfg.DO_CLOSE, 1)
        self._send_service("c")
        deadline = time.monotonic() + cfg.GRIP_TIMEOUT
        while time.monotonic() < deadline:
            if self.is_holding():
                time.sleep(cfg.GRIP_SETTLE)
                return True
            time.sleep(0.05)
        msg = "%.1f초 안에 잡힘 감지(DI%d)가 들어오지 않았습니다." % (cfg.GRIP_TIMEOUT, cfg.DI_GRIP)
        if self.strict:
            raise GearError(msg)
        self.log.warn(msg + " (검사 완화 모드라 계속 진행)")
        return False
