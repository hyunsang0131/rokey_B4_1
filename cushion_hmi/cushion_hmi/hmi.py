# 협동1 쿠션 진단 HMI (PyQt5)
#
#   ros2 run cushion_hmi hmi
#
# 미션 노드와 /cushion/* 토픽으로만 통신한다 (형식은 protocol.py).
# 로봇을 직접 움직이지 않으므로, 로봇이 동작하는 동안에도 화면이 멈추지 않는다.
#
# 주의: [정지 요청]은 미션 노드에 정지를 "요청"하는 소프트웨어 명령이다.
#       위급 상황에서는 반드시 티칭 펜던트나 비상정지 버튼을 쓴다.

import signal
import sys
import time

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QComboBox, QFrame, QGroupBox, QHBoxLayout, QLabel,
                             QMainWindow, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)

from cushion_hmi import protocol as P
from cushion_hmi.ros_bridge import RosBridge
from cushion_hmi.widgets import ForcePlot, ModuleGrid, StateBar

BTN_STYLE = """
QPushButton { padding: 6px 16px; border-radius: 5px; border: 1px solid #c9ced6;
              background: #ffffff; }
QPushButton:hover:enabled { background: #f3f4f6; }
QPushButton:disabled { color: #a0a7b2; background: #f5f6f8; }
QPushButton#start:enabled { background: #1f5f8b; color: white; border-color: #1f5f8b; }
QPushButton#stop:enabled { background: #dc2626; color: white; border-color: #dc2626; }
QPushButton#continue { background: #a86200; color: white; border-color: #a86200;
                       font-weight: bold; }
"""


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.status = {}
        self.last_status_time = None
        self.setWindowTitle('협동1 쿠션 진단 HMI')
        self.resize(1200, 780)
        self._build()

        bridge.status_received.connect(self.on_status)
        bridge.force_received.connect(self.on_force)
        bridge.log_received.connect(self.append_log)

        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self.refresh_link)
        self.link_timer.start(500)
        self.refresh_link()
        self.append_log('[HMI] 시작. 미션 노드의 /cushion/status 를 기다리는 중')

    # ------------------------------------------------------------------ 화면 구성
    def _build(self):
        root = QWidget()
        root.setStyleSheet(BTN_STYLE)
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(16, 12, 16, 12)
        v.setSpacing(10)

        # 상단: 제목, 연결 상태, 모드, 버튼
        top = QHBoxLayout()
        title = QLabel('매트리스 쿠션감 진단 · 모듈 교체')
        title.setFont(QFont(self.font().family(), 15, QFont.Bold))
        top.addWidget(title)
        top.addSpacing(16)
        self.link_label = QLabel()
        top.addWidget(self.link_label)
        top.addStretch(1)
        self.info_label = QLabel()
        self.info_label.setStyleSheet('color: #4b5563;')
        top.addWidget(self.info_label)
        top.addSpacing(16)
        top.addWidget(QLabel('모드'))
        self.mode_combo = QComboBox()
        for key, text in P.MODES:
            self.mode_combo.addItem(text, key)
        self.mode_combo.setCurrentIndex(P.MODE_KEYS.index('full'))
        top.addWidget(self.mode_combo)
        self.btn_start = QPushButton('시작')
        self.btn_start.setObjectName('start')
        self.btn_stop = QPushButton('정지 요청')
        self.btn_stop.setObjectName('stop')
        self.btn_stop.setToolTip('미션 노드에 정지를 요청합니다. 비상시에는 펜던트·비상정지 버튼을 쓰세요.')
        self.btn_reset = QPushButton('초기화')
        for b in (self.btn_start, self.btn_stop, self.btn_reset):
            top.addWidget(b)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_reset.clicked.connect(self.on_reset)
        v.addLayout(top)

        # 플로우 단계
        self.state_bar = StateBar()
        v.addWidget(self.state_bar)

        # 작업자 확인 배너 (수동 단계)
        self.prompt_frame = QFrame()
        self.prompt_frame.setStyleSheet(
            'QFrame { background: #fbefdc; border: 1px solid #e3b26b; border-radius: 6px; }'
            'QLabel { border: none; color: #6b3f00; font-weight: bold; }')
        ph = QHBoxLayout(self.prompt_frame)
        ph.setContentsMargins(14, 8, 8, 8)
        self.prompt_label = QLabel()
        ph.addWidget(self.prompt_label, 1)
        self.btn_continue = QPushButton('작업 완료 · 계속')
        self.btn_continue.setObjectName('continue')
        self.btn_continue.clicked.connect(self.on_continue)
        ph.addWidget(self.btn_continue)
        self.prompt_frame.hide()
        v.addWidget(self.prompt_frame)

        # 알람 배너
        self.alarm_label = QLabel()
        self.alarm_label.setStyleSheet(
            'background: #fee2e2; color: #991b1b; border: 1px solid #f19999;'
            'border-radius: 6px; padding: 8px 14px; font-weight: bold;')
        self.alarm_label.setWordWrap(True)
        self.alarm_label.hide()
        v.addWidget(self.alarm_label)

        # 가운데: 모듈 격자 | 그래프
        mid = QHBoxLayout()
        mid.setSpacing(12)
        g1 = QGroupBox('모듈 상태 (가로 2 × 세로 3)')
        l1 = QVBoxLayout(g1)
        self.grid = ModuleGrid()
        l1.addWidget(self.grid)
        mid.addWidget(g1, 4)

        g2 = QGroupBox('힘–변위 (압입 측정)')
        self.plot_box = g2
        l2 = QVBoxLayout(g2)
        self.plot = ForcePlot()
        l2.addWidget(self.plot, 1)
        self.readout = QLabel('모듈 —   깊이 — mm   Fz — N')
        self.readout.setStyleSheet('font-family: monospace; font-size: 13px; color: #1f2937;')
        l2.addWidget(self.readout)
        mid.addWidget(g2, 5)
        v.addLayout(mid, 1)

        # 아래: 로그
        g3 = QGroupBox('로그')
        l3 = QVBoxLayout(g3)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setStyleSheet('font-family: monospace; font-size: 12px;')
        self.log.setFixedHeight(150)
        l3.addWidget(self.log)
        v.addWidget(g3)

    # ------------------------------------------------------------------ 수신 처리
    def on_status(self, d):
        self.status = d
        self.last_status_time = time.monotonic()
        self.state_bar.set_state(d.get('state'))
        self.grid.set_modules(d.get('modules', []), d.get('current_module'))

        if d.get('wait_operator'):
            self.prompt_label.setText('작업자 확인 필요: ' + (d.get('prompt') or ''))
            self.prompt_frame.show()
            self.btn_continue.setEnabled(True)
        else:
            self.prompt_frame.hide()

        alarm = d.get('alarm') or ''
        self.alarm_label.setText('⚠ ' + alarm)
        self.alarm_label.setVisible(bool(alarm))

        if d.get('running') and d.get('mode') in P.MODE_KEYS:
            self.mode_combo.setCurrentIndex(P.MODE_KEYS.index(d['mode']))

        parts = []
        if d.get('control') is False:
            parts.append('표시 전용 (조작은 터미널에서)')
        if d.get('spares') is not None:
            parts.append(f'선반 재고 {d["spares"]}')
        if d.get('retry'):
            parts.append(f'재시도 {d["retry"]}')
        self.info_label.setText('   '.join(parts))
        self.refresh_link()

    def on_force(self, d):
        # 압입 측정은 z(깊이), 정지 측정은 t(시간)를 가로축으로 쓴다
        kind = 'depth' if 'z' in d else 'time'
        try:
            mid, fz = int(d['module']), float(d['fz'])
            x = float(d['z'] if kind == 'depth' else d['t'])
        except (KeyError, TypeError, ValueError):
            return
        self.plot.add_point(mid, x, fz, bool(d.get('reset')), kind)
        self.plot_box.setTitle('힘–변위 (압입 측정)' if kind == 'depth' else '힘 (정지 측정, 시간축)')
        xs = f'깊이 {x:6.2f} mm' if kind == 'depth' else f'시간 {x:6.2f} s'
        self.readout.setText(f'모듈 M{mid}   {xs}   Fz {fz:7.2f} N')

    def append_log(self, text):
        self.log.appendPlainText(time.strftime('%H:%M:%S  ') + text)

    def connected(self):
        return (self.last_status_time is not None
                and time.monotonic() - self.last_status_time < P.STATUS_TIMEOUT)

    def refresh_link(self):
        alive = self.connected()
        running = alive and bool(self.status.get('running'))
        # control=false 인 발행자(예: measure_teach_force)는 HMI 명령을 받지 않으므로 버튼을 끈다
        control = self.status.get('control', True) is not False
        source = self.status.get('source') or '미션 노드'
        if alive:
            self.link_label.setText(f'● {source} 연결됨')
            self.link_label.setStyleSheet('color: #22a05a; font-weight: bold;')
        elif self.last_status_time is None:
            self.link_label.setText('● 미션 노드 대기 중')
            self.link_label.setStyleSheet('color: #6b7280; font-weight: bold;')
        else:
            self.link_label.setText(f'● {source} 응답 없음')
            self.link_label.setStyleSheet('color: #dc2626; font-weight: bold;')
        self.btn_start.setEnabled(alive and control and not running)
        self.btn_reset.setEnabled(alive and control and not running)
        self.btn_stop.setEnabled(control and running)
        self.mode_combo.setEnabled(control and not running)

    # ------------------------------------------------------------------ 버튼
    def on_start(self):
        mode = self.mode_combo.currentData()
        self.plot.clear()
        self.bridge.send_cmd('start', mode=mode)
        self.append_log(f'[HMI] 시작 요청 (모드: {self.mode_combo.currentText()})')
        self.btn_start.setEnabled(False)

    def on_stop(self):
        self.bridge.send_cmd('stop')
        self.append_log('[HMI] 정지 요청')

    def on_reset(self):
        self.plot.clear()
        self.bridge.send_cmd('reset')
        self.append_log('[HMI] 초기화 요청')

    def on_continue(self):
        self.bridge.send_cmd('continue')
        self.append_log('[HMI] 작업 완료 · 계속')
        self.btn_continue.setEnabled(False)  # 다음 status 가 올 때까지 중복 클릭 방지


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = RosBridge()
    win = MainWindow(bridge)
    win.show()
    bridge.start()

    # Ctrl+C 로 종료되게: Qt 루프가 파이썬 시그널 처리 기회를 주도록 주기적으로 깨운다
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    wake = QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(200)

    code = app.exec_()
    bridge.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
