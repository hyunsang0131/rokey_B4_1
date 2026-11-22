# 협동-1 새 운전화면 (test 브랜치 cushion_hmi/cushion_hmi.pyqty, 2026-11-20) + 실제 공정 연동.
#
# 위쪽 클래스들(그래프, 위치 지도, 주문 미리보기, 알람 창, Coop1MainHMI)은 팀원 원본 그대로이고,
# 맨 아래 Coop1RosHMI 가 그 화면을 bed_inspect 공정(cushion_bridge 경유)에 연결한다:
#   - 침대/트레이 칸: 공정 맵 (기본 공정: 불량 빨강 / 정상 파랑 / 빈칸 흰색, 주문 모드: Hard 파랑 / Soft 초록 / Soft High 보라)
#     큐브를 옮길 때마다 공정이 보내는 상태로 바로 다시 칠한다
#   - 시작(기본 공정) / 요청(측정만) / 정지 / 초기화(비상정지 해제) / 비상정지 → 명령 서비스 (/cushion/command, 응답 확인)
#   - 웹 주문(127.0.0.1:5000) → 주문 리스트 → 미리보기 → "실행" 을 눌러야 주문 모드 시작 (시작되면 리스트에서 빠짐)
#   - 예외 상황(충돌 긴급 프로토콜, 재고 부족 큐브 탐색), 사람 확인(진행/취소), 공정 로그, 실제 힘 곡선
#   ros2 run cushion_hmi hmi          (예전 화면: ros2 run cushion_hmi hmi_classic)

import json
import os
import sys
import random
import math
from datetime import datetime
from PyQt5 import QtWidgets, QtCore, QtGui


# ==============================================================================
# 1. 실시간 접촉 강성(Stiffness [N/mm]) 그래프 위젯
# ==============================================================================
class RealtimeStiffnessGraph(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = [1.2 + random.uniform(0.0, 0.5) for _ in range(90)]
        self.max_val = 30.0
        self.limit_val = 25.0
        self.setMinimumHeight(170)

    def add_point(self, val):
        self.history.append(val)
        if len(self.history) > 90:
            self.history.pop(0)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # 라운드 화이트 배경
        rect = QtCore.QRectF(1, 1, w - 2, h - 2)
        painter.setBrush(QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#dbe4ee"), 1.5))
        painter.drawRoundedRect(rect, 10, 10)

        # 가로 계측 눈금선 및 우측 라벨
        painter.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        for i in range(1, 5):
            y = int((h * i) / 5)
            painter.setPen(QtGui.QPen(QtGui.QColor("#e2e8f0"), 1))
            painter.drawLine(8, y, w - 8, y)

            label_val = int(self.max_val * (1 - i / 5))
            painter.setPen(QtGui.QColor("#94a3b8"))
            painter.drawText(w - 28, y - 3, f"{label_val}")

        # 25.0 N/mm 과부하 임계선
        thresh_y = int(h - (self.limit_val / self.max_val) * (h - 28) - 12)
        pen_thresh = QtGui.QPen(QtGui.QColor("#ef4444"), 1.2, QtCore.Qt.DashLine)
        painter.setPen(pen_thresh)
        painter.drawLine(8, thresh_y, w - 8, thresh_y)

        painter.setPen(QtGui.QColor("#dc2626"))
        painter.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        painter.drawText(12, thresh_y - 4, f"LIMIT {self.limit_val:.1f} N/mm")

        # 접촉 강성 곡선 그리기
        if len(self.history) > 1:
            path = QtGui.QPainterPath()
            for idx, val in enumerate(self.history):
                x = 8 + (idx / (len(self.history) - 1)) * (w - 16)
                y = h - (min(val, self.max_val) / self.max_val) * (h - 28) - 12
                if idx == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor("#16a34a"), 2.2))
            painter.drawPath(path)

        # 상단 실시간 디지털 수치
        curr_val = self.history[-1] if self.history else 0.0
        painter.setPen(QtGui.QColor("#0f172a"))
        painter.setFont(QtGui.QFont("Noto Sans KR", 9, QtGui.QFont.Bold))
        painter.drawText(12, 20, f"CONTACT: {curr_val:.2f} N/mm")


# ==============================================================================
# 2. 물체 위치 추정 지도 프론트엔드 위젯 (상단 로봇 / 좌하단 침대 / 우하단 트레이)
# ==============================================================================
class PositionEstimationMap(QtWidgets.QWidget):
    """위에서 본 배치도: 상단 로봇 / 좌하단 침대 2x3 / 우하단 트레이 2x3 (트레이가 조금 작음).
    칸 번호는 왼쪽 배치도와 같이  1 2 / 3 4 / 5 6.  지금 작업 중인 칸은 강조하고 TCP 를 그 칸에 표시 (11-22 형님).
    (팀원 원본의 Bed#1 / Bed#4 / Tray#2 주황 점은 시연용 예시라 뺐음)"""
    BED = (0.08, 0.36, 0.36, 0.56)          # x, y, w, h (비율)
    TRAY = (0.56, 0.36, 0.30, 0.46)         # 윗선은 침대와 같게, 크기는 조금 작게

    def __init__(self, parent=None):
        super().__init__(parent)
        self.predicted_points = []
        self.tcp_pos = (0.50, 0.30)
        self.active = None                  # ('bed' | 'tray', 칸 번호 1~6) 또는 None
        self.setMinimumHeight(190)

    def update_state(self, tcp_ratio_xy, detected_list=None, active=None):
        self.tcp_pos = tcp_ratio_xy
        if detected_list is not None:
            self.predicted_points = detected_list
        self.active = active
        self.update()

    @classmethod
    def cell_center(cls, zone, n):
        """칸 번호(1~6) → 지도 비율 좌표 (가운데). 1 2 / 3 4 / 5 6."""
        x, y, w, h = cls.BED if zone == 'bed' else cls.TRAY
        r, c = divmod(n - 1, 2)
        pad_t = 0.16                         # 위쪽 이름 자리
        gh = h * (1 - pad_t)
        return (x + w * (0.25 + 0.5 * c), y + h * pad_t + gh * (r + 0.5) / 3)

    def _zone(self, painter, w, h, rect, fill, line, txt_col, name, zone):
        x, y, rw, rh = rect
        r = QtCore.QRectF(w * x, h * y, w * rw, h * rh)
        painter.setBrush(fill)
        painter.setPen(QtGui.QPen(line, 1.5))
        painter.drawRoundedRect(r, 8, 8)
        painter.setPen(txt_col)
        painter.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        painter.drawText(QtCore.QRectF(r.left() + 6, r.top() + 2, r.width() - 12, h * rh * 0.16), QtCore.Qt.AlignVCenter, name)
        pad_t = rh * 0.16
        cw, ch = rw / 2, rh * (1 - 0.16) / 3
        for n in range(1, 7):
            rr, cc = divmod(n - 1, 2)
            cell = QtCore.QRectF(w * (x + cw * cc) + 4, h * (y + pad_t + ch * rr) + 3, w * cw - 8, h * ch - 6)
            act = self.active if isinstance(self.active, list) else ([self.active] if self.active else [])
            on = (zone, n) in act
            painter.setBrush(QtGui.QColor(line.red(), line.green(), line.blue(), 110 if on else 28))
            painter.setPen(QtGui.QPen(line, 2.0 if on else 1.0))
            painter.drawRoundedRect(cell, 5, 5)
            painter.setPen(txt_col)
            painter.setFont(QtGui.QFont("Noto Sans KR", 9 if on else 8, QtGui.QFont.Bold))
            if on:                          # 작업 중인 칸: 가운데는 TCP 점 자리라 번호는 왼쪽 위에
                painter.drawText(cell.adjusted(5, 2, 0, 0), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, str(n))
            else:
                painter.drawText(cell, QtCore.Qt.AlignCenter, str(n))

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # 라운드 화이트 배경
        painter.setBrush(QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#dbe4ee"), 1.5))
        painter.drawRoundedRect(QtCore.QRectF(1, 1, w - 2, h - 2), 10, 10)

        # 좌표 격자
        painter.setPen(QtGui.QPen(QtGui.QColor("#f1f5f9"), 1))
        for i in range(1, 6):
            painter.drawLine(int(w * i / 6), 4, int(w * i / 6), h - 4)
            painter.drawLine(4, int(h * i / 6), w - 4, int(h * i / 6))

        self._zone(painter, w, h, self.BED, QtGui.QColor(2, 132, 199, 16), QtGui.QColor("#7dd3fc"),
                   QtGui.QColor("#0369a1"), "BED ZONE (2x3)", 'bed')
        self._zone(painter, w, h, self.TRAY, QtGui.QColor(217, 119, 6, 16), QtGui.QColor("#fcd34d"),
                   QtGui.QColor("#b45309"), "TRAY ZONE (2x3)", 'tray')

        # (시연용) 예측 물체 좌표 점
        for rx, ry, label in self.predicted_points:
            px, py = int(rx * w), int(ry * h)
            painter.setBrush(QtGui.QColor("#f97316"))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPoint(px, py), 5, 5)
            painter.setPen(QtGui.QColor("#334155"))
            painter.drawText(px + 8, py + 4, label)

        # 상단 중앙 ROBOT 원점 -> TCP 링크
        robot_x, robot_y = int(w * 0.50), int(h * 0.16)
        tcp_x, tcp_y = int(self.tcp_pos[0] * w), int(self.tcp_pos[1] * h)
        painter.setPen(QtGui.QPen(QtGui.QColor("#0284c7"), 2.4))
        painter.drawLine(robot_x, robot_y, tcp_x, tcp_y)
        painter.setBrush(QtGui.QColor("#0f172a"))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRoundedRect(QtCore.QRectF(robot_x - 26, robot_y - 11, 52, 22), 6, 6)
        painter.setPen(QtGui.QColor("#38bdf8"))
        painter.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        painter.drawText(QtCore.QRectF(robot_x - 26, robot_y - 11, 52, 22), QtCore.Qt.AlignCenter, "ROBOT")
        painter.setBrush(QtGui.QColor("#2563eb"))
        painter.drawEllipse(QtCore.QPoint(tcp_x, tcp_y), 6, 6)
        painter.setPen(QtGui.QColor("#1d4ed8"))
        painter.drawText(tcp_x + 9, tcp_y + 14, "TCP")


# ==============================================================================
# 3. 주문 침대 배치 상세 미리보기 팝업 다이얼로그
# ==============================================================================
class OrderPreviewDialog(QtWidgets.QDialog):
    def __init__(self, order_data, parent=None):
        super().__init__(parent)
        self.order_data = order_data
        self.execute_requested = False
        self.setWindowTitle(f"주문 침대 배치 상세 미리보기 - {order_data['id']}")
        self.setFixedSize(440, 470)
        self.setStyleSheet("""
            QDialog { background-color: #f8fafc; font-family: 'Noto Sans KR', sans-serif; }
            QLabel { color: #0f172a; }
        """)
        self._init_ui()

    def _init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        # 헤더 배너
        header = QtWidgets.QFrame()
        header.setStyleSheet("background-color: #0284c7; border-radius: 12px; padding: 8px;")
        h_layout = QtWidgets.QHBoxLayout(header)
        lbl_title = QtWidgets.QLabel("📋 주문 침대 배치 상세 미리보기")
        lbl_title.setStyleSheet("color: white; font-weight: bold; font-size: 14px;")
        lbl_id = QtWidgets.QLabel(self.order_data["id"])
        lbl_id.setStyleSheet("color: white; background-color: rgba(255,255,255,0.2); border-radius: 10px; padding: 3px 10px; font-weight: bold;")
        h_layout.addWidget(lbl_title)
        h_layout.addStretch()
        h_layout.addWidget(lbl_id)
        layout.addWidget(header)

        # 요약 정보
        cubes = self.order_data["cubes"]
        summary_text = (
            f"접수 시각: {self.order_data['time']}   |   "
            f"Hard {cubes.count('Hard')} · Soft {cubes.count('Soft')} · Soft High {cubes.count('Soft High')}"
        )
        lbl_summary = QtWidgets.QLabel(summary_text)
        lbl_summary.setStyleSheet("background-color: white; border: 1.5px solid #cbd5e1; border-radius: 10px; padding: 8px 12px; font-weight: bold; color: #0369a1;")
        layout.addWidget(lbl_summary)

        # 침대 2x3 배치 시각화 박스
        bed_frame = QtWidgets.QFrame()
        bed_frame.setStyleSheet("background-color: #e2e8f0; border: 2px solid #94a3b8; border-radius: 14px; padding: 10px;")
        grid = QtWidgets.QGridLayout(bed_frame)
        grid.setSpacing(8)

        color_map = {
            "Hard": ("#2563eb", "18.8N/mm / 25.0mm"),
            "Soft": ("#16a34a", "6.4N/mm / 25.0mm"),
            "Soft High": ("#7c3aed", "7.5N/mm / 35.0mm"),
        }

        for idx in range(6):
            num = idx + 1
            c_type = cubes[idx]
            bg, spec = color_map.get(c_type, ("#64748b", "0.0N/mm / 0.0mm"))
            cell = QtWidgets.QLabel(f"[{num}] {c_type}\n{spec}")
            cell.setAlignment(QtCore.Qt.AlignCenter)
            cell.setMinimumHeight(68)
            cell.setStyleSheet(f"""
                background-color: {bg};
                color: white;
                border-radius: 10px;
                font-weight: bold;
                font-size: 12px;
            """)
            grid.addWidget(cell, idx // 2, idx % 2)

        layout.addWidget(bed_frame)

        # 하단 버튼
        btn_row = QtWidgets.QHBoxLayout()
        btn_close = QtWidgets.QPushButton("닫기")
        btn_close.setMinimumHeight(40)
        btn_close.setStyleSheet("background-color: #e2e8f0; color: #334155; border-radius: 10px; font-weight: bold;")
        btn_close.clicked.connect(self.reject)

        btn_exec = QtWidgets.QPushButton("▶ 이 주문 바로 실행하기")
        btn_exec.setMinimumHeight(40)
        btn_exec.setStyleSheet("background-color: #16a34a; color: white; border-radius: 10px; font-weight: bold; font-size: 13px;")
        btn_exec.clicked.connect(self._on_execute)

        btn_row.addWidget(btn_close, 1)
        btn_row.addWidget(btn_exec, 2)
        layout.addLayout(btn_row)

    def _on_execute(self):
        self.execute_requested = True
        self.accept()


# ==============================================================================
# 4. 설비 예외상황(ALARM) 팝업 다이얼로그
# ==============================================================================
class ExceptionAlarmDialog(QtWidgets.QDialog):
    def __init__(self, err_code, description, timestamp, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"설비 예외상황 발생 [{err_code}]")
        self.setFixedSize(430, 290)
        self.setStyleSheet("QDialog { background-color: #ffffff; font-family: 'Noto Sans KR', sans-serif; }")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        header = QtWidgets.QLabel(f"🚨 설비 예외상황 발생 (ALARM)   [{err_code}]")
        header.setStyleSheet("background-color: #dc2626; color: white; border-radius: 10px; padding: 10px; font-weight: bold; font-size: 14px;")
        layout.addWidget(header)

        info = QtWidgets.QLabel(
            f"• 발생 시각: {timestamp}\n"
            f"• 제어 상태: 로봇 모션 인터록 정지 (LOCKED)\n\n"
            f"• 원인 상세:\n  {description}"
        )
        info.setWordWrap(True)
        info.setStyleSheet("background-color: #fef2f2; border: 1.5px solid #fca5a5; border-radius: 10px; padding: 12px; color: #991b1b; font-weight: bold;")
        layout.addWidget(info)

        btn_reset = QtWidgets.QPushButton("알람 확인 및 리셋 (RESET & RECOVER)")
        btn_reset.setMinimumHeight(42)
        btn_reset.setStyleSheet("background-color: #dc2626; color: white; border-radius: 10px; font-weight: bold; font-size: 13px;")
        btn_reset.clicked.connect(self.accept)
        layout.addWidget(btn_reset)


# ==============================================================================
# 5. 메인 HMI 윈도우 (화이트 라운드 베이스 & 실물 배치도 규격)
# ==============================================================================
class Coop1MainHMI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("협동-1 매트릭스 자동화 운전화면 [DOOSAN M0609 HMI]")
        self.resize(1440, 880)

        # 상태 데이터
        self.bed_data = {i: {"type": "미측정", "stiffness": 0.0, "height": 0.0} for i in range(1, 7)}
        self.tray_data = {i: {"type": "미측정", "stiffness": 0.0, "height": 0.0} for i in range(1, 7)}
        self.orders = []
        self.order_counter = 1
        self.swap_selection = None
        self.is_locked = False
        self.spike_val = 0.0
        self.sim_tick = 0

        self._apply_global_style()
        self._build_ui()

        # 초기 웹 주문 샘플 2건 추가 (리스트 등록만 하고 자동 실행하지 않음)
        self.receive_web_order(["Hard", "Soft", "Soft High", "Hard", "Soft", "Hard"])
        self.receive_web_order(["Soft High", "Soft High", "Hard", "Soft", "Hard", "Soft"])

        # 타이머 (시계 및 실시간 센서/위치 추정 스트림)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(80)

        self.add_log("협동-1 HMI 운전화면 초기화 완료. 웹 주문 및 사용자 실행 대기 중...")

    def _apply_global_style(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #eef2f6;
                font-family: 'Noto Sans KR', 'Pretendard', sans-serif;
            }
            QFrame#HmiPanel {
                background-color: #ffffff;
                border: 1.5px solid #cbd5e1;
                border-radius: 14px;
            }
            QFrame#HmiInset {
                background-color: #f8fafc;
                border: 1.5px solid #dbe4ee;
                border-radius: 12px;
            }
            QLabel#SectionTitle {
                color: #0f172a;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton {
                border-radius: 9px;
                font-weight: bold;
                padding: 6px 10px;
            }
            QListWidget, QTextEdit {
                background-color: #ffffff;
                border: 1.5px solid #dbe4ee;
                border-radius: 10px;
                color: #1e293b;
                padding: 4px;
            }
        """)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(8)

        # -------------------- 1. 상단 헤더 바 --------------------
        header_panel = QtWidgets.QFrame(objectName="HmiPanel")
        header_layout = QtWidgets.QHBoxLayout(header_panel)
        header_layout.setContentsMargins(14, 8, 14, 8)

        badge_logo = QtWidgets.QLabel("COOP-1 HMI")
        badge_logo.setStyleSheet("background-color: #0f172a; color: #38bdf8; border-radius: 11px; padding: 3px 10px; font-weight: bold;")
        lbl_header = QtWidgets.QLabel("운전화면  |  협동-1 매트릭스 자동화 [DOOSAN M0609]")
        lbl_header.setStyleSheet("font-size: 15px; font-weight: bold; color: #0f172a;")

        btn_sim_order = QtWidgets.QPushButton("+ 웹 주문 수신")
        btn_sim_order.setStyleSheet("background-color: #f1f5f9; color: #1e293b; border: 1px solid #cbd5e1;")
        btn_sim_order.clicked.connect(lambda: self.receive_web_order(
            [random.choice(["Hard", "Soft", "Soft High"]) for _ in range(6)]
        ))

        btn_sim_swap = QtWidgets.QPushButton("⇄ 빠른 교체 테스트")
        btn_sim_swap.setStyleSheet("background-color: #e0f2fe; color: #0369a1; border: 1px solid #bae6fd;")
        btn_sim_swap.clicked.connect(self.quick_swap_demo)

        btn_sim_err = QtWidgets.QPushButton("⚠ 예외테스트")
        btn_sim_err.setStyleSheet("background-color: #fee2e2; color: #b91c1c; border: 1px solid #fecaca;")
        btn_sim_err.clicked.connect(lambda: self.trigger_exception("ERR-305", "그리퍼 파지 강성 이상 및 목표 높이 불일치 감지 (Bed #2)"))

        self.lbl_status_pill = QtWidgets.QLabel("● IDLE")
        self.lbl_status_pill.setStyleSheet("background-color: #f8fafc; border: 1.5px solid #cbd5e1; border-radius: 12px; padding: 4px 12px; color: #b45309; font-weight: bold;")

        self.lbl_clock = QtWidgets.QLabel(datetime.now().strftime("%Y/%m/%d %H:%M:%S"))
        self.lbl_clock.setStyleSheet("background-color: #f1f5f9; border: 1.5px solid #cbd5e1; border-radius: 12px; padding: 4px 12px; color: #1e293b; font-weight: bold;")

        header_layout.addWidget(badge_logo)
        header_layout.addWidget(lbl_header)
        header_layout.addStretch()
        header_layout.addWidget(btn_sim_order)
        header_layout.addWidget(btn_sim_swap)
        header_layout.addWidget(btn_sim_err)
        header_layout.addWidget(self.lbl_status_pill)
        header_layout.addWidget(self.lbl_clock)
        root_layout.addWidget(header_panel)

        # -------------------- 2. 메인 2단 그리드 (좌측 67% / 우측 33%) --------------------
        main_grid = QtWidgets.QHBoxLayout()
        main_grid.setSpacing(8)

        # ==================== 좌측 영역 ====================
        left_col = QtWidgets.QVBoxLayout()
        left_col.setSpacing(8)

        # [좌측 상단] 작업영역 모니터링 (상단 중앙 로봇 / 좌하단 침대 / 우하단 교체 트레이)
        ws_panel = QtWidgets.QFrame(objectName="HmiPanel")
        ws_layout = QtWidgets.QVBoxLayout(ws_panel)
        ws_layout.setContentsMargins(12, 10, 12, 12)

        ws_top_bar = QtWidgets.QHBoxLayout()
        ws_title = QtWidgets.QLabel("■ 작업영역 모니터링 (실물 배치도 - FLOORPLAN)", objectName="SectionTitle")
        ws_legend = QtWidgets.QLabel("■ Hard   ■ Soft   ■ Soft High   |   셀 2개 순차 클릭 시 즉시 교체")
        ws_legend.setStyleSheet("color: #475569; font-size: 11px; font-weight: bold;")
        ws_top_bar.addWidget(ws_title)
        ws_top_bar.addStretch()
        ws_top_bar.addWidget(ws_legend)
        ws_layout.addLayout(ws_top_bar)

        floorplan_inset = QtWidgets.QFrame(objectName="HmiInset")
        fp_layout = QtWidgets.QVBoxLayout(floorplan_inset)
        fp_layout.setContentsMargins(14, 12, 14, 12)
        fp_layout.setSpacing(10)

        # 상단 중앙: 로봇 베이스
        robot_row = QtWidgets.QHBoxLayout()
        robot_row.addStretch()
        robot_box = QtWidgets.QFrame()
        robot_box.setMinimumWidth(340)
        robot_box.setStyleSheet("background-color: #ffffff; border: 2px solid #94a3b8; border-radius: 14px; padding: 6px 12px;")
        rb_layout = QtWidgets.QHBoxLayout(robot_box)
        rb_badge = QtWidgets.QLabel("로봇")
        rb_badge.setStyleSheet("background-color: #0f172a; color: #38bdf8; border-radius: 9px; padding: 4px 10px; font-weight: bold;")
        self.lbl_robot_detail = QtWidgets.QLabel("DOOSAN M0609  |  대기 중 (IDLE)")
        self.lbl_robot_detail.setStyleSheet("font-weight: bold; color: #0f172a;")
        self.lbl_tcp_coord = QtWidgets.QLabel("TCP: HOME")
        self.lbl_tcp_coord.setStyleSheet("color: #0284c7; font-weight: bold; font-size: 11px;")
        rb_layout.addWidget(rb_badge)
        rb_layout.addWidget(self.lbl_robot_detail)
        rb_layout.addStretch()
        rb_layout.addWidget(self.lbl_tcp_coord)
        robot_row.addWidget(robot_box)
        robot_row.addStretch()
        fp_layout.addLayout(robot_row)

        # 하단 좌측(세로형 침대 2x3) + 하단 우측(가로형 교체 트레이 2x3)
        zones_row = QtWidgets.QHBoxLayout()
        zones_row.setSpacing(18)

        # 좌하단: 침대 (BED 2x3)
        bed_col = QtWidgets.QVBoxLayout()
        lbl_bed_hdr = QtWidgets.QLabel("침대 (BED 2×3)")
        lbl_bed_hdr.setAlignment(QtCore.Qt.AlignCenter)
        lbl_bed_hdr.setStyleSheet("color: #0369a1; font-weight: bold; font-size: 13px;")
        bed_frame = QtWidgets.QFrame()
        bed_frame.setStyleSheet("background-color: #e2e8f0; border: 2px solid #94a3b8; border-radius: 14px; padding: 8px;")
        bed_grid = QtWidgets.QGridLayout(bed_frame)
        bed_grid.setSpacing(8)
        self.bed_buttons = {}
        for idx in range(6):
            num = idx + 1
            btn = QtWidgets.QPushButton(f"[{num}] 미측정\n0.0N/mm / 0.0mm")
            btn.setMinimumHeight(64)
            btn.clicked.connect(lambda _, n=num: self._on_cell_clicked("bed", n))
            bed_grid.addWidget(btn, idx // 2, idx % 2)
            self.bed_buttons[num] = btn
        bed_col.addWidget(lbl_bed_hdr)
        bed_col.addWidget(bed_frame)

        # 우하단: 교체 트레이 (STORAGE TRAY 2x3)
        tray_col = QtWidgets.QVBoxLayout()
        tray_col.addStretch()
        lbl_tray_hdr = QtWidgets.QLabel("교체 트레이 (STORAGE TRAY 2×3)")
        lbl_tray_hdr.setAlignment(QtCore.Qt.AlignCenter)
        lbl_tray_hdr.setStyleSheet("color: #b45309; font-weight: bold; font-size: 13px;")
        tray_frame = QtWidgets.QFrame()
        tray_frame.setStyleSheet("background-color: #e2e8f0; border: 2px solid #94a3b8; border-radius: 14px; padding: 8px;")
        tray_grid = QtWidgets.QGridLayout(tray_frame)
        tray_grid.setSpacing(8)
        self.tray_buttons = {}
        for idx in range(6):
            num = idx + 1
            btn = QtWidgets.QPushButton(f"[{num}] 미측정\n0.0N/mm / 0.0mm")
            btn.setMinimumHeight(52)
            btn.clicked.connect(lambda _, n=num: self._on_cell_clicked("tray", n))
            tray_grid.addWidget(btn, idx // 2, idx % 2)
            self.tray_buttons[num] = btn
        tray_col.addWidget(lbl_tray_hdr)
        tray_col.addWidget(tray_frame)

        zones_row.addLayout(bed_col, 5)
        zones_row.addLayout(tray_col, 6)
        fp_layout.addLayout(zones_row)
        ws_layout.addWidget(floorplan_inset)
        left_col.addWidget(ws_panel, 62)

        # [좌측 하단] 고객 주문 리스트 + 실시간 로그
        bottom_left_row = QtWidgets.QHBoxLayout()
        bottom_left_row.setSpacing(8)

        # 고객 주문 리스트 패널
        order_panel = QtWidgets.QFrame(objectName="HmiPanel")
        order_layout = QtWidgets.QVBoxLayout(order_panel)
        order_layout.setContentsMargins(10, 8, 10, 10)
        order_hdr = QtWidgets.QHBoxLayout()
        order_hdr.addWidget(QtWidgets.QLabel("■ 고객 주문 리스트 (클릭 시 배치 미리보기)", objectName="SectionTitle"))
        self.lbl_order_cnt = QtWidgets.QLabel("대기: 0건")
        self.lbl_order_cnt.setStyleSheet("color: #0369a1; font-weight: bold;")
        order_hdr.addStretch()
        order_hdr.addWidget(self.lbl_order_cnt)
        order_layout.addLayout(order_hdr)

        self.list_orders = QtWidgets.QListWidget()
        self.list_orders.itemClicked.connect(self._on_order_item_clicked)
        order_layout.addWidget(self.list_orders)

        btn_exec_order = QtWidgets.QPushButton("▶ 선택된 주문 실행하기")
        btn_exec_order.setMinimumHeight(38)
        btn_exec_order.setStyleSheet("background-color: #16a34a; color: white; font-size: 13px;")
        btn_exec_order.clicked.connect(self.execute_selected_order)
        order_layout.addWidget(btn_exec_order)

        # 실시간 로그 패널
        log_panel = QtWidgets.QFrame(objectName="HmiPanel")
        log_layout = QtWidgets.QVBoxLayout(log_panel)
        log_layout.setContentsMargins(10, 8, 10, 10)
        log_hdr = QtWidgets.QHBoxLayout()
        log_hdr.addWidget(QtWidgets.QLabel("■ 실시간 특징 감지 및 작업 로그", objectName="SectionTitle"))
        btn_clear_log = QtWidgets.QPushButton("지우기")
        btn_clear_log.setStyleSheet("background-color: #f1f5f9; color: #334155; border: 1px solid #cbd5e1; padding: 2px 8px;")
        btn_clear_log.clicked.connect(lambda: self.text_log.clear())
        log_hdr.addStretch()
        log_hdr.addWidget(btn_clear_log)
        log_layout.addLayout(log_hdr)

        self.text_log = QtWidgets.QTextEdit()
        self.text_log.setReadOnly(True)
        log_layout.addWidget(self.text_log)

        bottom_left_row.addWidget(order_panel, 5)
        bottom_left_row.addWidget(log_panel, 7)
        left_col.addLayout(bottom_left_row, 38)

        # ==================== 우측 영역 ====================
        right_col = QtWidgets.QVBoxLayout()
        right_col.setSpacing(8)

        # [우측 상단] 시스템 상시 제어반 고정
        ctrl_panel = QtWidgets.QFrame(objectName="HmiPanel")
        ctrl_layout = QtWidgets.QVBoxLayout(ctrl_panel)
        ctrl_layout.setContentsMargins(10, 8, 10, 10)
        ctrl_layout.addWidget(QtWidgets.QLabel("■ 시스템 상시 제어반 (FIXED PANEL)", objectName="SectionTitle"))

        ctrl_btns = QtWidgets.QHBoxLayout()
        ctrl_btns.setSpacing(6)
        btn_specs = [
            ("시작", "#2563eb", self.handle_start),
            ("정지", "#d97706", self.handle_stop),
            ("요청", "#475569", self.handle_request_probing),
            ("초기화", "#64748b", self.handle_reset),
            ("비상정지", "#dc2626", self.handle_estop),
        ]
        for label, bg, callback in btn_specs:
            b = QtWidgets.QPushButton(label)
            b.setMinimumHeight(40)
            b.setStyleSheet(f"background-color: {bg}; color: white; font-size: 13px;")
            b.clicked.connect(callback)
            ctrl_btns.addWidget(b)
        ctrl_layout.addLayout(ctrl_btns)
        right_col.addWidget(ctrl_panel, 1)

        # [우측 중단] 실시간 접촉 힘 그래프 (Stiffness [N/mm])
        force_panel = QtWidgets.QFrame(objectName="HmiPanel")
        force_layout = QtWidgets.QVBoxLayout(force_panel)
        force_layout.setContentsMargins(10, 8, 10, 10)
        f_hdr = QtWidgets.QHBoxLayout()
        f_hdr.addWidget(QtWidgets.QLabel("■ 실시간 접촉 힘 그래프 (Stiffness [N/mm])", objectName="SectionTitle"))
        btn_spike = QtWidgets.QPushButton("터치 강성 테스트")
        btn_spike.setStyleSheet("background-color: #dcfce7; color: #15803d; border: 1px solid #bbf7d0; padding: 2px 8px; font-size: 11px;")
        btn_spike.clicked.connect(lambda: setattr(self, "spike_val", 18.6))
        f_hdr.addStretch()
        f_hdr.addWidget(btn_spike)
        force_layout.addLayout(f_hdr)

        self.force_graph = RealtimeStiffnessGraph()
        force_layout.addWidget(self.force_graph, 1)          # 그래프가 패널 세로를 다 채우게 (11-22 형님)
        right_col.addWidget(force_panel, 5)                  # 힘 그래프 / 위치 지도 패널 같은 크기

        # [우측 하단] 물체 위치 추정 지도
        map_panel = QtWidgets.QFrame(objectName="HmiPanel")
        map_layout = QtWidgets.QVBoxLayout(map_panel)
        map_layout.setContentsMargins(10, 8, 10, 10)
        map_layout.addWidget(QtWidgets.QLabel("■ 물체 위치 추정 지도 (그리퍼 궤적 & 예측 좌표)", objectName="SectionTitle"))

        self.pos_map = PositionEstimationMap()
        map_layout.addWidget(self.pos_map, 1)
        right_col.addWidget(map_panel, 5)

        main_grid.addLayout(left_col, 67)
        main_grid.addLayout(right_col, 33)
        root_layout.addLayout(main_grid)

        self._refresh_matrix_ui()

    # ==========================================================================
    # 매트릭스 UI 갱신 및 빠른 셀 교체
    # ==========================================================================
    def _refresh_matrix_ui(self):
        style_map = {
            "Hard": ("#2563eb", "#ffffff"),
            "Soft": ("#16a34a", "#ffffff"),
            "Soft High": ("#7c3aed", "#ffffff"),
            "미측정": ("#ffffff", "#475569"),
        }
        for zone_name, data_dict, btn_dict in [
            ("bed", self.bed_data, self.bed_buttons),
            ("tray", self.tray_data, self.tray_buttons),
        ]:
            for num, btn in btn_dict.items():
                info = data_dict[num]
                bg, fg = style_map.get(info["type"], ("#ffffff", "#475569"))
                is_sel = (self.swap_selection == (zone_name, num))
                border = "3px solid #f59e0b" if is_sel else "1.5px solid rgba(15, 23, 42, 0.16)"
                btn.setStyleSheet(f"""
                    background-color: {bg};
                    color: {fg};
                    border: {border};
                    border-radius: 10px;
                    font-weight: bold;
                    font-size: 12px;
                """)
                btn.setText(f"[{num}] {info['type']}\n{info['stiffness']:.1f}N/mm / {info['height']:.1f}mm")

    def _on_cell_clicked(self, zone, num):
        if self.is_locked:
            self.trigger_exception("ERR-102", "설비가 알람 잠금 상태입니다. 먼저 알람을 해제하십시오.")
            return
        if self.swap_selection is None:
            self.swap_selection = (zone, num)
            self._refresh_matrix_ui()
            self.add_log(f"교체 원본 선택: {zone.upper()}[{num}]")
            return

        src_zone, src_num = self.swap_selection
        if (src_zone, src_num) == (zone, num):
            self.swap_selection = None
            self._refresh_matrix_ui()
            return

        src_dict = self.bed_data if src_zone == "bed" else self.tray_data
        dst_dict = self.bed_data if zone == "bed" else self.tray_data
        src_dict[src_num], dst_dict[num] = dst_dict[num].copy(), src_dict[src_num].copy()
        self.swap_selection = None
        self._refresh_matrix_ui()
        self.add_log(f"데이터 즉시 스왑 완료: {src_zone.upper()}[{src_num}] ⇄ {zone.upper()}[{num}]")

    def quick_swap_demo(self):
        if self.bed_data[1]["type"] == "미측정":
            self.handle_request_probing()
        self.bed_data[1], self.tray_data[2] = self.tray_data[2].copy(), self.bed_data[1].copy()
        self._refresh_matrix_ui()
        self.add_log("빠른 교체 테스트: BED[1] ⇄ TRAY[2] 강성·높이 데이터 즉시 갱신")

    # ==========================================================================
    # 웹 주문 수신, 배치 미리보기 팝업, 수동 실행 승인
    # ==========================================================================
    def receive_web_order(self, cubes):
        order_id = f"ORD-{self.order_counter:03d}"
        self.order_counter += 1
        order_obj = {
            "id": order_id,
            "time": datetime.now().strftime("%H:%M:%S"),
            "cubes": cubes
        }
        self.orders.append(order_obj)
        self._refresh_order_list()
        self.add_log(f"웹 주문 수신 ({order_id}) - 사용자 실행 승인 대기 중")

    def _refresh_order_list(self):
        self.list_orders.clear()
        self.lbl_order_cnt.setText(f"대기: {len(self.orders)}건")
        for order in self.orders:
            item = QtWidgets.QListWidgetItem(f"[{order['id']}] ({order['time']})  구성: {' / '.join(order['cubes'])}")
            item.setData(QtCore.Qt.UserRole, order)
            self.list_orders.addItem(item)

    def _on_order_item_clicked(self, item):
        order_data = item.data(QtCore.Qt.UserRole)
        if not order_data:
            return
        self.add_log(f"주문 배치 미리보기 열람: [{order_data['id']}]")
        dlg = OrderPreviewDialog(order_data, self)
        if dlg.exec_() == QtWidgets.QDialog.Accepted and dlg.execute_requested:
            self.execute_selected_order()

    def execute_selected_order(self):
        if self.is_locked:
            self.trigger_exception("ERR-102", "설비가 알람 잠금 상태입니다.")
            return
        current_item = self.list_orders.currentItem()
        if not current_item:
            QtWidgets.QMessageBox.information(self, "알림", "실행할 고객 주문을 리스트에서 먼저 선택하세요.")
            return

        order = current_item.data(QtCore.Qt.UserRole)
        self._set_robot_status("RUNNING", f"주문 실행 중 ({order['id']})", "#16a34a", "#15803d")
        self.add_log(f"사용자 실행 승인 → [{order['id']}] 목표 침대 배치 시퀀스 시작")

        for idx, c_type in enumerate(order["cubes"]):
            stiff = 18.8 if c_type == "Hard" else (6.4 if c_type == "Soft" else 7.5)
            height = 35.0 if c_type == "Soft High" else 25.0
            self.bed_data[idx + 1] = {"type": c_type, "stiffness": stiff, "height": height}

        self.spike_val = 16.4
        self._refresh_matrix_ui()

        self.orders = [o for o in self.orders if o["id"] != order["id"]]
        self._refresh_order_list()

    # ==========================================================================
    # 상시 제어반 및 예외 알람 팝업
    # ==========================================================================
    def _set_robot_status(self, badge, desc, pill_color, text_color):
        self.lbl_status_pill.setText(f"● {badge}")
        self.lbl_status_pill.setStyleSheet(
            f"background-color: #f8fafc; border: 1.5px solid #cbd5e1; border-radius: 12px; padding: 4px 12px; color: {pill_color}; font-weight: bold;"
        )
        self.lbl_robot_detail.setText(f"DOOSAN M0609  |  {desc}")
        self.lbl_robot_detail.setStyleSheet(f"font-weight: bold; color: {text_color};")

    def handle_start(self):
        if self.is_locked:
            self.trigger_exception("ERR-102", "알람 해제 후 시작할 수 있습니다.")
            return
        self._set_robot_status("RUNNING", "자동 운전 중 (RUNNING)", "#16a34a", "#15803d")
        self.add_log("시스템 제어: 로봇 운전 시작 (START)")

    def handle_stop(self):
        if not self.is_locked:
            self._set_robot_status("PAUSED", "일시 정지 (PAUSED)", "#eab308", "#b45309")
            self.add_log("시스템 제어: 로봇 일시 정지 (STOP)")

    def handle_request_probing(self):
        if self.is_locked:
            return
        self._set_robot_status("PROBING", "촉각 강성·높이 스캔 중...", "#0284c7", "#0369a1")
        self.add_log("요청: 침대(BED) 및 교체 트레이(TRAY) 초기 강성(N/mm)·높이 프로빙 실행")

        bed_preset = [("Hard", 18.7, 25.0), ("Soft", 6.3, 24.8), ("Soft High", 7.4, 35.1),
                      ("Hard", 19.1, 25.1), ("Soft", 6.1, 24.9), ("Hard", 18.5, 25.0)]
        tray_preset = [("Soft High", 7.6, 35.0), ("Hard", 18.9, 25.0), ("Soft", 6.5, 24.7),
                       ("Soft High", 7.3, 35.2), ("Hard", 18.6, 25.0), ("Soft", 6.2, 24.9)]

        for i in range(1, 7):
            self.bed_data[i] = {"type": bed_preset[i - 1][0], "stiffness": bed_preset[i - 1][1], "height": bed_preset[i - 1][2]}
            self.tray_data[i] = {"type": tray_preset[i - 1][0], "stiffness": tray_preset[i - 1][1], "height": tray_preset[i - 1][2]}

        self.spike_val = 19.1
        self._refresh_matrix_ui()

    def handle_reset(self):
        self.is_locked = False
        self.swap_selection = None
        for i in range(1, 7):
            self.bed_data[i] = {"type": "미측정", "stiffness": 0.0, "height": 0.0}
            self.tray_data[i] = {"type": "미측정", "stiffness": 0.0, "height": 0.0}
        self._refresh_matrix_ui()
        self._set_robot_status("IDLE", "대기 중 (IDLE)", "#eab308", "#b45309")
        self.add_log("시스템 및 매트릭스 데이터 전체 초기화 완료 (RESET)")

    def handle_estop(self):
        self.trigger_exception("E-STOP", "작업자 비상정지(E-STOP) 버튼이 눌렸습니다. 모든 서보 모션이 즉시 차단됩니다.")

    def trigger_exception(self, err_code, description):
        self.is_locked = True
        self._set_robot_status("ALARM", "비상 정지 / 알람 잠금", "#dc2626", "#dc2626")
        ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        self.add_log(f"🚨 [예외발생 {err_code}] {description}")

        dlg = ExceptionAlarmDialog(err_code, description, ts, self)
        dlg.exec_()
        self.is_locked = False
        self._set_robot_status("IDLE", "대기 중 (IDLE)", "#eab308", "#b45309")
        self.add_log("알람 확인 및 인터록 복구 완료 (IDLE)")

    def add_log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.text_log.append(f"[{ts}] {msg}")

    def _on_tick(self):
        self.sim_tick += 1
        self.lbl_clock.setText(datetime.now().strftime("%Y/%m/%d %H:%M:%S"))

        val = 1.1 + random.uniform(0.0, 0.5)
        if self.spike_val > 0.5:
            val = self.spike_val + random.uniform(-0.2, 0.2)
            self.spike_val *= 0.86
        self.force_graph.add_point(val)

        t = self.sim_tick * 0.05
        rx = 0.50 + math.cos(t) * 0.22
        ry = 0.58 + math.sin(t * 1.3) * 0.16
        self.pos_map.update_state((rx, ry))
        self.lbl_tcp_coord.setText(f"TCP: ({int((rx - 0.5) * 400)}, {int((ry - 0.16) * 400)})")


# ==============================================================================
# 6. 실제 공정 연동 (bed_inspect cushion_bridge ↔ 이 화면)
# ==============================================================================
from cushion_hmi import protocol as P   # noqa: E402

CELL_STYLE = {          # 종류 → (배경, 글자)
    # 주문 모드: 웹 주문 페이지와 같은 색 (2026-11-21 형님) — 딱딱 = 어두운 남색, 부드러움 = 밝은 베이지, 높은 부드러움 = 조금 진한 베이지
    'Hard': ('#4f6479', '#ffffff'),
    'Soft': ('#f3ebdf', '#3b3127'),
    'Soft High': ('#ddcdb5', '#3b3127'),
    '정상': ('#2563eb', '#ffffff'),          # 기본 공정: 딱딱 = 정상 (파랑)
    '불량': ('#dc2626', '#ffffff'),          # 기본 공정: 말랑 = 불량 (빨강)
    '빈칸': ('#ffffff', '#475569'),
    '미측정': ('#ffffff', '#94a3b8'),
}
H_NORMAL, H_TALL = 20.0, 29.0           # 보통 큐브 / 키 큰 큐브 높이 [mm]
STOCK_COUNTS = (('Soft High', '높은 말랑', 2), ('Soft', '일반 말랑', 2), ('Hard', '일반 딱딱', 6))
CUBE_OF = {'H': 'Hard', 'S': 'Soft', 'T': 'Soft High'}


def web_slots_to_cubes(slots):
    """웹 주문 {'Slot n': {'stiffness', 'height'}} → ['Hard', 'Soft', 'Soft High', ...] (화면 표시용)."""
    out = []
    for n in range(1, 7):
        v = slots[f'Slot {n}']
        out.append('Soft High' if v['height'] == 'HIGH' else 'Soft' if v['stiffness'] == '부드러움' else 'Hard')
    return out


ORDER_HISTORY = os.path.expanduser('~/ws_cobot_pjt/bed_data/order_history.json')   # 주문 내역 (HMI 를 껐다 켜도 남음)
STATUS_COLOR = {'대기': '#b45309', '실행 중': '#15803d', '완료': '#0369a1', '완료 (확인 필요)': '#a16207',
                '오류': '#dc2626', '중지': '#64748b'}


class OrderDetailDialog(OrderPreviewDialog):
    """주문 상세 (언제든 다시 열 수 있음): 팀원 미리보기 모양 + 웹 색 + 상태/시간. 실행은 '대기' 주문만."""
    def __init__(self, order_data, parent=None):
        super().__init__(order_data, parent)
        self.setWindowTitle(f"주문 상세 - {order_data['id']}")
        self.setFixedSize(470, 600)

    def _init_ui(self):
        o = self.order_data
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        header = QtWidgets.QFrame()
        header.setStyleSheet("background-color: #1f2937; border-radius: 12px; padding: 8px;")
        hl = QtWidgets.QHBoxLayout(header)
        t = QtWidgets.QLabel("📋 주문 침대 배치 상세")
        t.setStyleSheet("color: white; font-weight: bold; font-size: 14px;")
        st = QtWidgets.QLabel(o.get('status', '대기'))
        st.setStyleSheet(f"color: white; background-color: {STATUS_COLOR.get(o.get('status'), '#64748b')};"
                         " border-radius: 10px; padding: 3px 10px; font-weight: bold;")
        oid = QtWidgets.QLabel(o['id'])
        oid.setStyleSheet("color: white; background-color: rgba(255,255,255,0.2); border-radius: 10px; padding: 3px 10px; font-weight: bold;")
        hl.addWidget(t); hl.addStretch(); hl.addWidget(st); hl.addWidget(oid)
        layout.addWidget(header)
        cubes = o["cubes"]
        times = f"접수 {o.get('date', '')} {o['time']}".strip()
        if o.get('start'):
            times += f"  ·  시작 {o['start']}"
        if o.get('end'):
            times += f"  ·  끝 {o['end']}"
        info = QtWidgets.QLabel(f"{times}\n딱딱 {cubes.count('Hard')} · 부드러움 {cubes.count('Soft')} · "
                                f"높은 부드러움 {cubes.count('Soft High')}" + (f"\n결과: {o['result']}" if o.get('result') else ''))
        info.setWordWrap(True)
        info.setStyleSheet("background-color: white; border: 1.5px solid #cbd5e1; border-radius: 10px; padding: 8px 12px;"
                           " font-weight: bold; color: #334155;")
        layout.addWidget(info)
        bed_frame = QtWidgets.QFrame()
        bed_frame.setStyleSheet("background-color: #6b4f3a; border-radius: 14px; padding: 10px;")   # 침대 프레임 (갈색)
        grid = QtWidgets.QGridLayout(bed_frame)
        grid.setSpacing(8)
        spec = {'Hard': '딱딱 · 약 12N/mm', 'Soft': '부드러움 · 약 7N/mm', 'Soft High': '부드러움 · 약 7N/mm\nHIGH'}
        for idx in range(6):
            c = cubes[idx]
            bg, fg = CELL_STYLE.get(c, ('#64748b', '#ffffff'))
            cell = QtWidgets.QLabel(f"#{idx + 1}\n{spec.get(c, c)}")
            cell.setAlignment(QtCore.Qt.AlignCenter)
            cell.setMinimumHeight(68)
            cell.setStyleSheet(f"background-color: {bg}; color: {fg}; border-radius: 8px; font-weight: bold; font-size: 12px;")
            grid.addWidget(cell, idx // 2, idx % 2)
        layout.addWidget(bed_frame)
        row = QtWidgets.QHBoxLayout()
        close = QtWidgets.QPushButton("닫기")
        close.setMinimumHeight(40)
        close.setStyleSheet("background-color: #e2e8f0; color: #334155; border-radius: 10px; font-weight: bold;")
        close.clicked.connect(self.reject)
        row.addWidget(close, 1)
        if o.get('status', '대기') == '대기':
            run = QtWidgets.QPushButton("▶ 이 주문 실행하기")
            run.setMinimumHeight(40)
            run.setStyleSheet("background-color: #16a34a; color: white; border-radius: 10px; font-weight: bold; font-size: 13px;")
            run.clicked.connect(self._on_execute)
            row.addWidget(run, 2)
        layout.addLayout(row)


class ForceGraph(RealtimeStiffnessGraph):
    """실제 누르기 힘 곡선 [N] (공정이 칸 하나를 다 누르면 그 곡선을 보내 줌). 팀원 그래프 모양 그대로, 단위만 N."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = []
        self.max_val, self.limit_val = 40.0, 30.0
        self.title = '대기 중'

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        painter.setBrush(QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#dbe4ee"), 1.5))
        painter.drawRoundedRect(QtCore.QRectF(1, 1, w - 2, h - 2), 10, 10)
        painter.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        for i in range(1, 5):
            y = int((h * i) / 5)
            painter.setPen(QtGui.QPen(QtGui.QColor("#e2e8f0"), 1))
            painter.drawLine(8, y, w - 8, y)
            painter.setPen(QtGui.QColor("#94a3b8"))
            painter.drawText(w - 28, y - 3, f"{int(self.max_val * (1 - i / 5))}")
        thresh_y = int(h - (self.limit_val / self.max_val) * (h - 28) - 12)
        painter.setPen(QtGui.QPen(QtGui.QColor("#ef4444"), 1.2, QtCore.Qt.DashLine))
        painter.drawLine(8, thresh_y, w - 8, thresh_y)
        painter.setPen(QtGui.QColor("#dc2626"))
        painter.drawText(12, thresh_y - 4, f"목표 힘 {self.limit_val:.0f} N")
        if len(self.history) > 1:
            path = QtGui.QPainterPath()
            for idx, val in enumerate(self.history):
                x = 8 + (idx / (len(self.history) - 1)) * (w - 16)
                y = h - (min(val, self.max_val) / self.max_val) * (h - 28) - 12
                path.moveTo(x, y) if idx == 0 else path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor("#16a34a"), 2.2))
            painter.drawPath(path)
        painter.setPen(QtGui.QColor("#0f172a"))
        painter.setFont(QtGui.QFont("Noto Sans KR", 9, QtGui.QFont.Bold))
        cur = self.history[-1] if self.history else 0.0
        painter.drawText(12, 20, f"{self.title}   Fz {cur:.1f} N")


class MultiForceGraph(QtWidgets.QWidget):
    """침대 1~6번 누르기 힘-깊이 곡선을 한 그래프에 쌓아서 보여 준다 (2026-11-21 형님).
    칸을 누를 때마다 그 칸 곡선만 새로 바뀌고 나머지는 남는다. 빈칸은 범례에 '빈칸' 으로만 표시."""
    COLORS = ['#2563eb', '#dc2626', '#f59e0b', '#16a34a', '#7c3aed', '#0891b2']

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(190)
        self.clear()

    def clear(self):
        self.curves = {}            # 칸 → [(깊이 mm, 힘 N)]
        self.info = {}              # 칸 → {'type': S/H/T/E/?, 'k': ...}
        self.active = None
        self.f_max, self.f_target = 40.0, 30.0
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setBrush(QtGui.QColor("#ffffff"))
        p.setPen(QtGui.QPen(QtGui.QColor("#dbe4ee"), 1.5))
        p.drawRoundedRect(QtCore.QRectF(1, 1, w - 2, h - 2), 10, 10)
        lw = 118                                    # 오른쪽 범례 폭
        x0, y0, x1, y1 = 34, 12, w - lw - 8, h - 24
        pts = [pt for c in self.curves.values() for pt in c]
        d_max = max([3.0] + [d for d, _ in pts]) * 1.05
        p.setFont(QtGui.QFont("Noto Sans KR", 7))
        for i in range(5):                          # 가로 눈금 (힘)
            f = self.f_max * i / 4
            y = y1 - (y1 - y0) * i / 4
            p.setPen(QtGui.QPen(QtGui.QColor("#eef2f6"), 1))
            p.drawLine(int(x0), int(y), int(x1), int(y))
            p.setPen(QtGui.QColor("#94a3b8"))
            p.drawText(4, int(y) + 4, f"{f:.0f}")
        for i in range(5):                          # 세로 눈금 (깊이)
            x = x0 + (x1 - x0) * i / 4
            p.setPen(QtGui.QColor("#94a3b8"))
            p.drawText(int(x) - 8, h - 8, f"{d_max * i / 4:.1f}")
        p.drawText(int(x1) - 52, int(y1) - 6, "깊이 [mm]")
        p.drawText(x0 + 4, y0 + 10, "Fz [N]")
        yt = y1 - (y1 - y0) * self.f_target / self.f_max
        p.setPen(QtGui.QPen(QtGui.QColor("#ef4444"), 1.0, QtCore.Qt.DashLine))
        p.drawLine(int(x0), int(yt), int(x1), int(yt))
        for mid, c in sorted(self.curves.items()):
            if len(c) < 2:
                continue
            path = QtGui.QPainterPath()
            for n, (d, f) in enumerate(c):
                x = x0 + (x1 - x0) * min(d, d_max) / d_max
                y = y1 - (y1 - y0) * max(0.0, min(f, self.f_max)) / self.f_max
                path.moveTo(x, y) if n == 0 else path.lineTo(x, y)
            p.setPen(QtGui.QPen(QtGui.QColor(self.COLORS[(mid - 1) % 6]), 3.0 if mid == self.active else 1.8))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawPath(path)
        p.setFont(QtGui.QFont("Noto Sans KR", 8, QtGui.QFont.Bold))
        for i in range(1, 7):                       # 범례: 칸마다 상태
            y = 16 + (i - 1) * ((h - 24) / 6)
            inf = self.info.get(i, {})
            t = inf.get('type')
            if t == 'E':
                txt, col = f"{i}번  빈칸", '#94a3b8'
            elif i in self.curves and len(self.curves[i]) > 1:
                k = inf.get('k')
                txt, col = f"{i}번  " + (f"k {k:.2f}" if k else "측정됨"), self.COLORS[i - 1]
            else:
                txt, col = f"{i}번  대기", '#cbd5e1'
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor(col))
            p.drawRoundedRect(QtCore.QRectF(w - lw, y, 12, 12), 3, 3)
            p.setPen(QtGui.QColor('#0f172a' if col != '#cbd5e1' else '#94a3b8'))
            p.drawText(int(w - lw + 18), int(y + 11), txt + (" ◀" if i == self.active else ""))


class CubeSearchLiveWindow(QtWidgets.QWidget):
    """예외 상황 2 (재고 부족) 때 큐브 탐지 과정을 실시간으로 보여 주는 창 (11-22 형님).
    pjt_test1 cube_search 의 /cube_search/live 를 그린다:
      start(탐색 영역·큐브 크기) / touch(터치 위치·닿음) / estimate(추정 중심·각도·후보) / stage·log(단계 글) / result / fail
    위에서 본 그림 (카메라 기준, 11-22): 가로 = 베이스 y (오른쪽이 +y), 세로 = 베이스 x (아래쪽이 +x)."""

    def __init__(self, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.setWindowTitle("큐브 탐지 실시간 (위치 추정)")
        self.resize(560, 640)
        self.setStyleSheet("QWidget { background-color: #ffffff; font-family: 'Noto Sans KR', sans-serif; }")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        self.lbl_head = QtWidgets.QLabel("🔍 큐브 탐지 (위치 추정) 대기")
        self.lbl_head.setStyleSheet("background-color: #0f172a; color: #e2e8f0; border-radius: 8px; padding: 8px;"
                                    " font-weight: bold; font-size: 14px;")
        lay.addWidget(self.lbl_head)
        self.canvas = _SearchCanvas(self)
        lay.addWidget(self.canvas, 1)
        self.lbl_stage = QtWidgets.QLabel("")
        self.lbl_stage.setWordWrap(True)
        self.lbl_stage.setStyleSheet("color: #0f172a; font-weight: bold; font-size: 12px;")
        lay.addWidget(self.lbl_stage)
        self.txt = QtWidgets.QPlainTextEdit()
        self.txt.setReadOnly(True)
        self.txt.setMaximumHeight(130)
        self.txt.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; font-size: 11px;")
        lay.addWidget(self.txt)

    def reset(self, why=''):
        self.canvas.reset()
        self.lbl_head.setText("🔍 재고 부족 → 큐브 탐지 준비 중" + (f"  ({why})" if why else ''))
        self.lbl_stage.setText("실 고정해제 후 탐지를 시작합니다")
        self.txt.clear()

    def on_event(self, d):
        t = d.get('type')
        c = self.canvas
        if t == 'start':
            c.reset(d.get('area'), d.get('cube'))
            self.lbl_head.setText(f"🔍 큐브 탐지 중 — {d.get('method', '')}")
        elif t == 'touch':
            c.touches.append((d.get('x'), d.get('y'), d.get('hit')))
            self.lbl_head.setText(f"🔍 큐브 탐지 중 — 터치 {d.get('n', len(c.touches))}번")
        elif t == 'estimate':
            c.est = (d.get('cx'), d.get('cy'), d.get('th'), d.get('sp'), d.get('sa'))
            c.pts = d.get('pts') or []
        elif t in ('stage', 'log'):
            text = d.get('text', '')
            if t == 'stage':
                self.lbl_stage.setText(text)
            self.txt.appendPlainText(text)
        elif t == 'result':
            c.result = (d.get('cx'), d.get('cy'), d.get('th'), d.get('z_top'))
            c.pts = []
            el = d.get('elapsed') or 0
            self.lbl_head.setText(f"✅ 찾음: 중심 ({d.get('cx', 0):.1f}, {d.get('cy', 0):.1f}) mm, {d.get('th', 0):.1f}°  "
                                  f"· 터치 {d.get('touches')}번 · {el / 60:.1f}분")
            for w in d.get('warnings') or []:
                self.txt.appendPlainText(f"⚠ {w}")
        elif t == 'fail':
            self.lbl_head.setText(f"❌ 탐지 실패: {d.get('text', '')}")
        c.update()


class _SearchCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(320)
        self.reset()

    def reset(self, area=None, cube=None):
        self.area = area or {'x_min': 204.81, 'x_max': 344.81, 'y_min': -73.94, 'y_max': 76.06}
        self.cube = cube or {'long': 64.0, 'short': 41.0}
        self.touches, self.pts, self.est, self.result = [], [], None, None
        self.update()

    def _map(self):
        a = self.area
        w, h, m = self.width(), self.height(), 28
        sx = (w - 2 * m) / max(1e-6, a['y_max'] - a['y_min'])
        sy = (h - 2 * m) / max(1e-6, a['x_max'] - a['x_min'])
        s = min(sx, sy)
        ox = m + ((w - 2 * m) - s * (a['y_max'] - a['y_min'])) / 2
        oy = m + ((h - 2 * m) - s * (a['x_max'] - a['x_min'])) / 2

        def to(x, y):           # 베이스 (x, y) → 화면. 11-22 형님: 카메라 기준으로 180도 돌림 → 오른쪽 = +y, 아래쪽 = +x
            return QtCore.QPointF(ox + (y - a['y_min']) * s, oy + (x - a['x_min']) * s)
        return to, s

    def _cube_poly(self, to, cx, cy, th):
        L, S = self.cube.get('long', 64.0) / 2, self.cube.get('short', 41.0) / 2
        c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
        pts = []
        for u, v in ((L, S), (-L, S), (-L, -S), (L, -S)):
            pts.append(to(cx + u * c - v * s_, cy + u * s_ + v * c))
        return QtGui.QPolygonF(pts)

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        to, s = self._map()
        a = self.area
        tl, br = to(a['x_max'], a['y_max']), to(a['x_min'], a['y_min'])
        p.setBrush(QtGui.QColor("#f8fafc"))
        p.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 1.5, QtCore.Qt.DashLine))
        rect = QtCore.QRectF(tl, br).normalized()
        p.drawRect(rect)
        p.setPen(QtGui.QColor("#64748b"))
        p.setFont(QtGui.QFont("Noto Sans KR", 8))
        p.drawText(QtCore.QPointF(rect.left(), rect.top() - 6), f"탐색 영역 x {a['x_min']:.0f}~{a['x_max']:.0f}, y {a['y_min']:.0f}~{a['y_max']:.0f}")
        # 후보 (파티클)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(124, 58, 237, 70))
        for x, y in self.pts:
            p.drawEllipse(to(x, y), 2.2, 2.2)
        # 추정
        if self.est and self.est[0] is not None and not self.result:
            cx, cy, th = self.est[0], self.est[1], self.est[2] or 0.0
            p.setBrush(QtGui.QColor(249, 115, 22, 40))
            p.setPen(QtGui.QPen(QtGui.QColor("#f97316"), 1.5, QtCore.Qt.DashLine))
            p.drawPolygon(self._cube_poly(to, cx, cy, th))
        # 결과
        if self.result and self.result[0] is not None:
            cx, cy, th = self.result[0], self.result[1], self.result[2] or 0.0
            p.setBrush(QtGui.QColor(22, 163, 74, 60))
            p.setPen(QtGui.QPen(QtGui.QColor("#16a34a"), 2.2))
            p.drawPolygon(self._cube_poly(to, cx, cy, th))
            p.drawEllipse(to(cx, cy), 3, 3)
        # 터치
        for i, (x, y, hit) in enumerate(self.touches):
            if x is None or y is None:
                continue
            q = to(x, y)
            col = "#dc2626" if hit else ("#94a3b8" if hit is False else "#cbd5e1")
            p.setBrush(QtGui.QColor(col))
            p.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1))
            r = 4.5 if i == len(self.touches) - 1 else 3.2
            p.drawEllipse(q, r, r)
        # 범례
        p.setFont(QtGui.QFont("Noto Sans KR", 8))
        y0 = self.height() - 8
        for k, (col, name) in enumerate((("#dc2626", "닿음"), ("#94a3b8", "안 닿음"), ("#7c3aed", "후보"),
                                         ("#f97316", "추정"), ("#16a34a", "결과"))):
            x0 = 10 + k * 90
            p.setBrush(QtGui.QColor(col))
            p.setPen(QtCore.Qt.NoPen)
            p.drawEllipse(QtCore.QPointF(x0 + 4, y0 - 4), 4, 4)
            p.setPen(QtGui.QColor("#334155"))
            p.drawText(QtCore.QPointF(x0 + 12, y0), name)


class Coop1RosHMI(Coop1MainHMI):
    def __init__(self, bridge=None):
        self.bridge = bridge
        self.robot = None
        self.robot_time = 0.0
        self.defect_mode = False
        self.estopped = False
        self._estop_acked = False
        self.started_order = None
        self.stock_k = {'Soft High': None, 'Soft': None, 'Hard': None}
        self._last_em = None
        self._em_dlg = None
        self._wait_box = None
        self._last_alarm = None
        super().__init__()
        # 팀원 화면의 시연용 샘플 주문은 실물에서 잘못 실행될 수 있어 지운다. 대신 저장된 주문 내역을 불러온다
        self.orders = self._load_history() if bridge is not None else []
        self.order_counter = len(self.orders) + 1
        self.running_order = None
        self._refresh_order_list()
        self._add_stock_line()
        self.view_mode = 'process'
        self._last_maps = (None, None)
        self._add_mode_switch()
        if bridge is not None:
            self._hide_demo_buttons()
            self._use_real_graph()
            self.text_log.clear()
            bridge.status_received.connect(self.on_robot_status)
            bridge.force_received.connect(self.on_robot_force)
            bridge.log_received.connect(lambda t: self.add_log(f"[공정] {t}"))
            if hasattr(bridge, 'search_live'):
                bridge.search_live.connect(self.on_search_live)
            bridge.order_config_received.connect(self.on_order_config)
            if hasattr(bridge, 'cmd_result'):
                bridge.cmd_result.connect(self.on_cmd_result)
            self.add_log("실제 공정 연결 모드: 공정(bed_process) 상태를 기다립니다.")

    # ── 화면 준비 ──
    # ── 공정 모드 / 주문 모드 (2026-11-21 형님) ──
    def _add_mode_switch(self):
        """헤더에 [공정 모드] [주문 모드] 전환. 공정 모드: 말랑(높은 말랑 포함) = 불량 빨강, 딱딱 = 정상 파랑.
        주문 모드: 딱딱 남색 / 부드러움 밝은 베이지 / 높은 부드러움 진한 베이지. 공정을 시작하면 그 모드로 자동 전환."""
        box = QtWidgets.QFrame()
        box.setStyleSheet("background-color: #f1f5f9; border: 1.5px solid #cbd5e1; border-radius: 12px;")
        lay = QtWidgets.QHBoxLayout(box)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(3)
        self.mode_buttons = {}
        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)
        for key, text in (('process', '공정 모드'), ('order', '주문 모드')):
            b = QtWidgets.QPushButton(text)
            b.setCheckable(True)
            b.setMinimumWidth(92)
            b.clicked.connect(lambda _, k=key: self.set_view_mode(k, by_user=True))
            group.addButton(b)
            lay.addWidget(b)
            self.mode_buttons[key] = b
        hl = self.lbl_status_pill.parentWidget().layout()
        hl.insertWidget(hl.indexOf(self.lbl_status_pill), box)
        self.set_view_mode('process')

    def set_view_mode(self, mode, by_user=False):
        if by_user and self.robot is not None and self.robot.get('running'):
            run_mode = 'order' if self.robot.get('mode') == 'order' else 'process'
            if mode != run_mode:
                self.add_log("공정이 진행 중이라 모드를 바꿀 수 없습니다 (진행 중인 공정의 모드로 표시).")
                mode = run_mode
        self.view_mode = mode
        self.defect_mode = mode == 'process'
        for k, b in self.mode_buttons.items():
            on = k == mode
            b.setChecked(on)
            color = '#dc2626' if k == 'process' else '#4f6479'
            b.setStyleSheet(f"background-color: {color if on else '#ffffff'}; color: {'#ffffff' if on else '#334155'};"
                            " border-radius: 9px; font-weight: bold; padding: 5px 10px;")
        self._update_legend()
        bed, tray = self._last_maps
        if bed:
            self.bed_data.update(self._cells_from_map(bed))
        if tray:
            self.tray_data.update(self._cells_from_map(tray))
        self._refresh_matrix_ui()
        if hasattr(self, 'lbl_summary'):
            self._update_summary(bed, tray)
        if by_user:
            self.add_log(f"화면 모드: {'공정 모드 (말랑 = 불량)' if mode == 'process' else '주문 모드 (딱딱 / 부드러움 / 높은 부드러움)'}")

    def _hide_demo_buttons(self):
        for b in self.findChildren(QtWidgets.QPushButton):
            if b.text() in ("+ 웹 주문 수신", "⇄ 빠른 교체 테스트", "⚠ 예외테스트", "터치 강성 테스트"):
                b.hide()

    def _use_real_graph(self):
        old = self.force_graph
        self.force_graph = MultiForceGraph()
        self.force_graph.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        old.parentWidget().layout().replaceWidget(old, self.force_graph)
        old.deleteLater()
        for lbl in self.findChildren(QtWidgets.QLabel):
            if lbl.text().startswith("■ 실시간 접촉 힘 그래프"):
                lbl.setText("■ 실시간 누르기 힘 그래프 (침대 1~6번, Fz [N] - 깊이)")
            if lbl.text().startswith("■ Hard   ■ Soft"):
                self.lbl_legend = lbl
        self._update_legend()

    def _update_legend(self):
        if getattr(self, 'lbl_legend', None) is None:
            return
        if self.defect_mode:
            self.lbl_legend.setText("<span style='color:#2563eb'>■ 정상</span>  <span style='color:#dc2626'>■ 불량</span>"
                                    "  □ 빈칸  |  공정 모드 (말랑·높은 말랑 = 불량)")
        else:
            self.lbl_legend.setText("<span style='color:#4f6479'>■ 딱딱 (Hard)</span>  <span style='color:#cbb89a'>■ 부드러움 (Soft)</span>"
                                    "  <span style='color:#a8916f'>■ 높은 부드러움 (Soft High)</span>  □ 빈칸  |  주문 모드")

    def _add_stock_line(self):
        self.lbl_stock = QtWidgets.QLabel()
        self.lbl_stock.setStyleSheet("color: #334155; font-weight: bold; font-size: 11px;")
        self.lbl_summary = QtWidgets.QLabel("침대: 미측정  |  트레이: 미측정")
        self.lbl_summary.setStyleSheet("color: #0f172a; font-weight: bold; font-size: 12px;")
        panel = self.bed_buttons[1].parentWidget()
        while panel is not None and panel.objectName() != "HmiInset":
            panel = panel.parentWidget()
        if panel is not None:
            panel.layout().addWidget(self.lbl_summary)
            panel.layout().addWidget(self.lbl_stock)
        self._update_stock_line()

    def _update_stock_line(self):
        parts = []
        for key, name, n in STOCK_COUNTS:
            k = self.stock_k.get(key)
            parts.append(f"{name} {n}개 (k {'미측정' if k is None else f'{k:.2f}'})")
        self.lbl_stock.setText("실리콘 블록 재고:  " + "  ·  ".join(parts))

    def robot_connected(self):
        import time as _t
        return self.robot is not None and _t.monotonic() - self.robot_time < P.STATUS_TIMEOUT

    # ── 칸 그리기 (공정 맵 → 색) ──
    def _cells_from_map(self, cells):
        out = {}
        for i, c in enumerate(cells or [], 1):
            t = c.get('type')
            if t == 'E':
                name, h = '빈칸', 0.0
            elif t == '?':
                name, h = '미측정', 0.0
            elif self.defect_mode:
                name, h = ('불량' if t in ('S', 'T') else '정상'), (H_TALL if t == 'T' else H_NORMAL)
            else:
                name, h = CUBE_OF.get(t, '미측정'), (H_TALL if t == 'T' else H_NORMAL)
            out[i] = {'type': name, 'stiffness': c.get('k') or 0.0, 'height': h}
        return out

    def _refresh_matrix_ui(self):
        for zone_name, data_dict, btn_dict in [("bed", self.bed_data, self.bed_buttons),
                                               ("tray", self.tray_data, self.tray_buttons)]:
            for num, btn in btn_dict.items():
                info = data_dict[num]
                bg, fg = CELL_STYLE.get(info["type"], ("#ffffff", "#475569"))
                active = getattr(self, '_active', None) == (zone_name, num)
                is_sel = (self.swap_selection == (zone_name, num))
                border = ("3px solid #f59e0b" if is_sel else "3px solid #0ea5e9" if active
                          else "1.5px solid rgba(15, 23, 42, 0.16)")
                btn.setStyleSheet(f"background-color: {bg}; color: {fg}; border: {border}; border-radius: 10px;"
                                  f" font-weight: bold; font-size: 12px;")
                if info["type"] in ("빈칸", "미측정"):
                    btn.setText(f"[{num}] {info['type']}")
                else:
                    btn.setText(f"[{num}] {info['type']}\n{info['stiffness']:.2f}N/mm / {info['height']:.1f}mm")

    def _on_cell_clicked(self, zone, num):
        if self.bridge is not None:          # 실물 연결 중에는 화면에서 칸을 바꾸지 않는다 (실제와 달라짐)
            info = (self.bed_data if zone == "bed" else self.tray_data)[num]
            self.add_log(f"{'침대' if zone == 'bed' else '트레이'} {num}번: {info['type']}, "
                         f"k {info['stiffness']:.2f} N/mm, 높이 {info['height']:.1f} mm")
            return
        super()._on_cell_clicked(zone, num)

    # ── 공정 상태 ──
    def on_robot_status(self, d):
        import time as _t
        self.robot, self.robot_time = d, _t.monotonic()
        step, name = d.get('step') or 0, d.get('step_name') or ''
        st = d.get('state')
        label = P.STATE_LABELS.get(st, st)
        em = d.get('emergency')
        if self.estopped:
            self._set_robot_status("E-STOP", "비상정지 (서보 꺼짐) — 초기화로 해제", "#dc2626", "#dc2626")
        elif em:
            self._set_robot_status("ALARM", f"{em.get('text')} · {em.get('what') or ''}", "#dc2626", "#dc2626")
        elif st == 'FAULT':
            self._set_robot_status("ALARM", f"이상: {d.get('alarm') or ''}", "#dc2626", "#dc2626")
        elif d.get('running'):
            self._set_robot_status("RUNNING", f"{step}/9단계 {name} · {label}", "#16a34a", "#15803d")
        elif st == 'DONE':
            self._set_robot_status("DONE", "완료", "#0284c7", "#0369a1")
        else:
            self._set_robot_status("IDLE", f"대기 중 ({label})", "#eab308", "#b45309")
        # 기본 공정(주문 모드가 아닌 것): 말랑 = 불량 빨강, 딱딱 = 정상 파랑
        bed, tray = d.get('bed_map'), d.get('tray_map')
        self._last_maps = (bed or self._last_maps[0], tray or self._last_maps[1])
        running = bool(d.get('running'))
        if running and not getattr(self, '_was_running', False):
            self.force_graph.clear()                    # 새 공정: 그래프를 비우고 1번부터 다시 쌓는다
        self._was_running = running
        if bed:
            self.force_graph.info = {i: c for i, c in enumerate(bed, 1)}
            self.force_graph.update()
        if d.get('running') and d.get('mode'):          # 진행 중인 공정의 모드로 화면을 맞춘다
            want = 'order' if d.get('mode') == 'order' else 'process'
            if want != self.view_mode:
                self.set_view_mode(want)
                self.add_log(f"화면 모드 자동 전환: {'주문 모드' if want == 'order' else '공정 모드'}")
        if bed:
            self.bed_data.update(self._cells_from_map(bed))
        if tray:
            self.tray_data.update(self._cells_from_map(tray))
        cm = d.get('current_module')
        self._active = ('bed', cm) if (d.get('running') and cm) else None
        self._refresh_matrix_ui()
        self._update_summary(bed, tray)
        self._update_stock_k(bed, tray)
        self._update_pos_map(d)
        self._check_started_order(d)
        self._show_emergency(em)
        self._show_operator_wait(d)
        if d.get('alarm') and d.get('alarm') != self._last_alarm:
            self._last_alarm = d['alarm']
            self.add_log(f"⚠ {d['alarm']}")

    def _update_summary(self, bed, tray):
        def text(cells, is_bed):
            if not cells:
                return '미측정'
            n = {}
            for c in cells:
                n[c['type']] = n.get(c['type'], 0) + 1
            soft, hard, empty = n.get('S', 0) + n.get('T', 0), n.get('H', 0), n.get('E', 0)
            if self.defect_mode and is_bed:
                s = f"정상 {hard} · 불량 {soft}"
            else:
                s = f"딱딱 {hard} · 말랑 {soft}" + (f" (높은 {n['T']})" if n.get('T') else '')
            return s + (f" · 빈칸 {empty}" if empty else '')
        self.lbl_summary.setText(f"침대: {text(bed, True)}   |   트레이: {text(tray, False)}")

    def _update_stock_k(self, bed, tray):
        for t, key in (('T', 'Soft High'), ('S', 'Soft'), ('H', 'Hard')):
            ks = [c['k'] for c in (bed or []) + (tray or []) if c['type'] == t and c.get('k')]
            if ks:
                self.stock_k[key] = sum(ks) / len(ks)
        self._update_stock_line()

    def _update_pos_map(self, d):
        """위치 지도: 지금 작업 중인 침대 칸에 TCP 표시 (위치를 따로 받지 않으므로 칸 기준), 아니면 로봇 쪽."""
        cm = d.get('current_module') if d.get('running') else None
        rk = d.get('current_rack') if d.get('running') else None
        mv = d.get('current_move') if d.get('running') else None
        zone = {'bed': 'bed', 'rack': 'tray'}
        active = None
        if mv and mv.get('src') and mv.get('dst'):
            # 블록 옮기는 중: 가져오는 칸과 놓는 칸을 같이 강조, TCP 는 놓는 칸 쪽
            cells = [(zone.get(k), int(n)) for k, n in (mv['src'], mv['dst']) if zone.get(k)]
            active = cells
            if cells:
                pos = PositionEstimationMap.cell_center(*cells[-1])
            else:
                pos = (0.50, 0.30)
            names = {'bed': '침대', 'tray': '트레이'}
            self.lbl_tcp_coord.setText('TCP: ' + ' → '.join(f'{names[z]} {n}번' for z, n in cells))
        elif rk:
            active = ('tray', int(rk))
            pos = PositionEstimationMap.cell_center('tray', int(rk))
            self.lbl_tcp_coord.setText(f"TCP: 트레이 {rk}번")
        elif cm:
            active = ('bed', int(cm))
            pos = PositionEstimationMap.cell_center('bed', int(cm))
            self.lbl_tcp_coord.setText(f"TCP: 침대 {cm}번")
        else:
            pos = (0.50, 0.30)
            self.lbl_tcp_coord.setText("TCP: HOME" if not d.get('running') else "TCP: 이동 중")
        self.pos_map.update_state(pos, [], active)

    def _on_tick(self):
        if self.bridge is None:
            return super()._on_tick()
        self.lbl_clock.setText(datetime.now().strftime("%Y/%m/%d %H:%M:%S"))
        if self.robot is not None and not self.robot_connected():
            self._set_robot_status("NO LINK", "공정 응답 없음 (bed_process / cushion_bridge 확인)", "#64748b", "#475569")

    # ── 실제 힘 곡선 ──
    def on_robot_force(self, d):
        """누르기 곡선 점: 칸마다 따로 쌓는다. 새로 누른 칸(reset)만 지우고 다시 그린다."""
        try:
            mid, z, fz = int(d['module']), float(d['z']), float(d['fz'])
        except (KeyError, TypeError, ValueError):
            return
        g = self.force_graph
        if d.get('reset') or mid not in g.curves:
            g.curves[mid] = []
            g.active = mid
            self.add_log(f"침대 {mid}번 누르기 힘 곡선 수신")
        g.curves[mid].append((z, fz))
        if not hasattr(self, '_graph_timer'):           # 점이 몰려 와도 0.1초에 한 번만 다시 그린다
            self._graph_timer = QtCore.QTimer(self)
            self._graph_timer.setSingleShot(True)
            self._graph_timer.timeout.connect(lambda: self.force_graph.update())
        self._graph_timer.start(100)

    # ── 명령 (서비스, 응답 확인) ──
    CMD_KO = {'start': '시작', 'stop': '정지', 'continue': '진행', 'cancel': '취소',
              'estop': '비상정지', 'estop_reset': '비상정지 해제'}

    def _send(self, cmd, **kw):
        if self.bridge is None:
            return False
        self.bridge.send_cmd(cmd, **kw)
        return True

    def on_cmd_result(self, r):
        name = self.CMD_KO.get(r.get('cmd'), r.get('cmd'))
        ok = r.get('accepted')
        if r.get('cmd') == 'estop' and ok:
            self._estop_acked = True
        if r.get('cmd') == 'estop_reset' and ok:
            self.estopped = False
        self.add_log(f"{'응답' if ok else '응답 확인 못 함' if ok is None else '거절'} [{name}] {r.get('message')}")
        if ok is False:
            box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, f"{name} 명령",
                                        f"{name} 명령이 처리되지 않았습니다.\n\n{r.get('message')}",
                                        QtWidgets.QMessageBox.Ok, self)
            box.setModal(False)
            box.show()
            self._cmd_box = box

    def handle_start(self):                 # 시작: 공정 모드 = 기본 공정, 주문 모드 = 리스트에서 고른 주문 실행
        if self.bridge is None:
            return super().handle_start()
        if self.view_mode == 'order':
            return self.execute_selected_order()
        if self.estopped:
            self.add_log("비상정지 상태입니다. 초기화로 해제한 뒤 시작하세요.")
            return
        self._send('start', mode='repair')
        self.add_log("시작 요청: 기본 공정 (측정 → 판정 → 커버 → 교체 → 재측정)")

    def handle_request_probing(self):       # 요청 = 측정만 (침대 강성 측정, 교체 없음)
        if self.bridge is None:
            return super().handle_request_probing()
        if self.estopped:
            return
        self._send('start', mode='diagnose')
        self.add_log("요청: 침대 강성 측정만 (교체 없음)")

    def handle_stop(self):
        if self.bridge is None:
            return super().handle_stop()
        self._send('stop')
        self.add_log("정지 요청 → 공정")

    def handle_reset(self):
        if self.bridge is None:
            return super().handle_reset()
        if self.estopped:
            ans = QtWidgets.QMessageBox.question(
                self, "비상정지 해제", "로봇 주변 안전을 확인했나요?\n\n해제하면 서보를 다시 켭니다 (로봇은 움직이지 않음).\n"
                "공정은 '시작'으로 다시 시작해야 합니다.")
            if ans == QtWidgets.QMessageBox.Yes:
                self._send('estop_reset')
                self.add_log("비상정지 해제 요청 (서보 켜기)")
            return
        self.swap_selection = None
        self.text_log.clear()
        self.add_log("화면 초기화 (공정 상태는 그대로 다시 표시됩니다)")

    def handle_estop(self):
        if self.bridge is None:
            return super().handle_estop()
        # 공정에 비상정지 (공정이 정지 표시를 먼저 세운 뒤 빠른 정지 + 서보 끄기) → 1.5초 안에 응답 없으면 로봇에 직접
        self.estopped = True
        self._estop_acked = False
        self._send('estop')
        self.add_log("🚨 비상정지 요청 → 공정 (빠른 정지 + 서보 끄기)")
        self._set_robot_status("E-STOP", "비상정지 (서보 꺼짐) — 초기화로 해제", "#dc2626", "#dc2626")
        QtCore.QTimer.singleShot(1500, self._estop_fallback)
        dlg = ExceptionAlarmDialog("E-STOP", "작업자 비상정지 버튼: 로봇에 빠른 정지 + 서보 끄기를 보냈습니다.\n"
                                   "  안전 확인 후 '초기화'로 해제(서보 켜기)하세요.\n"
                                   "  ※ 소프트웨어 정지입니다. 위급하면 펜던트/물리 비상정지를 누르세요.",
                                   datetime.now().strftime("%Y/%m/%d %H:%M:%S"), self)
        dlg.setModal(False)
        dlg.show()
        self._estop_dlg = dlg

    def _estop_fallback(self):
        if self._estop_acked or self.bridge is None:
            return
        sent = self.bridge.estop_direct()
        self.add_log(f"공정 응답 없음 → 로봇에 직접: {' + '.join(sent) or '로봇 서비스도 없음 (펜던트 비상정지를 누르세요)'}")

    # ── 예외 상황 / 사람 확인 ──
    # ── 큐브 탐지 실시간 창 (예외 상황 2) ──
    def _search_window(self):
        if getattr(self, '_search_win', None) is None:
            self._search_win = CubeSearchLiveWindow(self)
        return self._search_win

    def _place_search_window(self):
        """예외 알람 창과 탐지 창을 나란히(알람 왼쪽, 탐지 오른쪽) 붙여 화면 안에 보이게 놓는다 (11-22 형님)."""
        win = self._search_window()
        scr = (self.screen() or QtWidgets.QApplication.primaryScreen()).availableGeometry()
        alarm = self._em_dlg if (self._em_dlg is not None and self._em_dlg.isVisible()) else None
        gap = 8
        aw = alarm.frameGeometry().width() if alarm else 0
        # 둘이 화면 폭에 들어가게 탐지 창 폭을 맞춤
        ww = min(560, max(380, scr.width() - aw - gap - 20))
        wh = min(640, scr.height() - 40)
        win.resize(ww, wh)
        total = aw + (gap if alarm else 0) + win.frameGeometry().width()
        g = self.frameGeometry()
        x = g.center().x() - total // 2
        y = g.top() + 60
        x = max(scr.left() + 4, min(x, scr.right() - total - 4))
        y = max(scr.top() + 4, min(y, scr.bottom() - max(wh, alarm.frameGeometry().height() if alarm else 0) - 4))
        if alarm:
            alarm.move(x, y)
            win.move(x + aw + gap, y)
        else:
            win.move(x, y)
        win.show()
        win.raise_()
        if alarm:
            alarm.raise_()

    def on_search_live(self, d):
        win = self._search_window()
        if d.get('type') == 'start' or not win.isVisible():
            self._place_search_window()
        win.on_event(d)
        if d.get('type') == 'result':
            self.add_log(f"큐브 탐지 결과: 중심 ({d.get('cx', 0):.1f}, {d.get('cy', 0):.1f}), {d.get('th', 0):.1f}°, "
                         f"터치 {d.get('touches')}번")
        elif d.get('type') == 'fail':
            self.add_log(f"큐브 탐지 실패: {d.get('text')}")

    def _show_emergency(self, em):
        key = None if not em else (em.get('kind'), em.get('what'), em.get('retry'))
        if key == self._last_em:
            return
        self._last_em = key
        if not em:
            if self._em_dlg is not None:
                self._em_dlg.close()
                self._em_dlg = None
                self.add_log("예외 상황 처리 끝 → 하던 공정을 이어서 합니다")
            return
        if em.get('kind') == 'collision':
            code = "COLLISION"
            desc = (f"충돌로 인한 긴급 프로토콜\n  작업: {em.get('what')}\n  충돌 위치: {em.get('pos')}\n"
                    "  정지 → 안정 대기 → 반대 방향 후퇴 → 홈 → 하던 작업 다시")
        else:
            code = "STOCK-LOW"
            # 11-22 형님: 원인 상세는 '재고 부족' (자세한 내용은 로그)
            desc = "재고 부족"
        self.add_log(f"🚨 [{code}] " + desc.replace("\n", " ")
                     + (f" — {em.get('text')} {em.get('what') or ''}" if code == "STOCK-LOW" else ""))
        if self._em_dlg is not None:
            self._em_dlg.close()
        dlg = ExceptionAlarmDialog(code, desc, datetime.now().strftime("%Y/%m/%d %H:%M:%S"), self)
        dlg.setModal(False)                 # 공정은 알아서 복구하므로 창이 공정을 막지 않게
        dlg.show()
        self._em_dlg = dlg
        if code == "STOCK-LOW":
            # 예외 상황 2: 알람 창 옆에 실시간 탐지 창 (11-22 형님)
            self._search_window().reset(em.get('text') or '')
            self._place_search_window()

    def _show_operator_wait(self, d):
        if not d.get('wait_operator'):
            if self._wait_box is not None:
                self._wait_box.close()
                self._wait_box = None
            return
        if self._wait_box is not None:
            return
        box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Question, "확인 필요", d.get('prompt') or "진행할까요?",
                                    QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel, self)
        box.button(QtWidgets.QMessageBox.Ok).setText("진행")
        box.button(QtWidgets.QMessageBox.Cancel).setText("취소")
        box.setModal(False)

        def done(r):
            self._wait_box = None
            self._send('continue' if r == QtWidgets.QMessageBox.Ok else 'cancel')
            self.add_log("확인: " + ("진행" if r == QtWidgets.QMessageBox.Ok else "취소"))
        box.finished.connect(done)
        box.show()
        self._wait_box = box

    # ── 웹 주문 → 리스트 → 실행 ──
    # 주문 내역: 대기 → 실행 중 → 완료/오류/중지. 실행한 주문도 리스트에 남아 언제든 눌러서 다시 볼 수 있다 (2026-11-21 형님)
    def _load_history(self):
        try:
            with open(ORDER_HISTORY) as f:
                orders = json.load(f)
        except (OSError, ValueError):
            return []
        for o in orders:
            if o.get('status') == '실행 중':          # HMI 가 꺼질 때 실행 중이던 것
                o['status'], o['result'] = '중지', 'HMI 를 다시 켜서 결과를 모름'
        return orders[-200:]

    def _save_history(self):
        try:
            os.makedirs(os.path.dirname(ORDER_HISTORY), exist_ok=True)
            with open(ORDER_HISTORY, 'w') as f:
                json.dump(self.orders[-200:], f, ensure_ascii=False, indent=1)
        except OSError as e:
            self.add_log(f"주문 내역 저장 실패: {e}")

    def _refresh_order_list(self):
        sel = self.list_orders.currentItem()
        sel_id = sel.data(QtCore.Qt.UserRole)['id'] if sel and sel.data(QtCore.Qt.UserRole) else None
        self.list_orders.clear()
        waiting = sum(1 for o in self.orders if o.get('status', '대기') == '대기')
        self.lbl_order_cnt.setText(f"대기 {waiting}건 · 전체 {len(self.orders)}건")
        names = {'Hard': '딱딱', 'Soft': '부드러움', 'Soft High': '높은 부드러움'}
        for o in reversed(self.orders):            # 최근 주문이 위
            stt = o.get('status', '대기')
            item = QtWidgets.QListWidgetItem(f"[{stt}] {o['id']} ({o['time']})  {' / '.join(names.get(c, c) for c in o['cubes'])}")
            item.setForeground(QtGui.QColor(STATUS_COLOR.get(stt, '#334155')))
            f = item.font(); f.setBold(stt in ('대기', '실행 중')); item.setFont(f)
            item.setData(QtCore.Qt.UserRole, o)
            self.list_orders.addItem(item)
            if o['id'] == sel_id:
                self.list_orders.setCurrentItem(item)

    def _on_order_item_clicked(self, item):
        o = item.data(QtCore.Qt.UserRole)
        if not o:
            return
        dlg = OrderDetailDialog(o, self)
        if dlg.exec_() == QtWidgets.QDialog.Accepted and dlg.execute_requested:
            self.execute_selected_order()

    def on_order_config(self, data):
        try:
            slots = P.normalize_order_slots(data.get("slots"))
        except ValueError as exc:
            self.add_log(f"주문 거부: {exc}")
            return
        order_id = data.get("order_id") or f"ORD-{self.order_counter:03d}"
        if any(o['id'] == order_id for o in self.orders):
            return
        self.order_counter += 1
        self.orders.append({"id": order_id, "date": datetime.now().strftime("%Y-%m-%d"),
                            "time": datetime.now().strftime("%H:%M:%S"), "status": "대기",
                            "cubes": web_slots_to_cubes(slots), "slots": slots})
        self._save_history()
        self._refresh_order_list()
        self.add_log(f"웹 주문 수신 ({order_id}) - '실행' 을 눌러야 시작합니다")

    def execute_selected_order(self):
        if self.bridge is None:
            return super().execute_selected_order()
        item = self.list_orders.currentItem()
        if not item:
            QtWidgets.QMessageBox.information(self, "알림", "실행할 고객 주문을 리스트에서 먼저 선택하세요.")
            return
        order = item.data(QtCore.Qt.UserRole)
        if self.view_mode != 'order':
            self.set_view_mode('order')
            self.add_log("주문 실행 → 주문 모드로 전환")
        if order.get('status', '대기') != '대기':
            QtWidgets.QMessageBox.information(self, "주문 실행", f"이 주문은 '{order.get('status')}' 상태입니다. "
                                              "대기 중인 주문만 실행할 수 있습니다.")
            return
        if self.estopped:
            self.add_log("비상정지 상태입니다. 초기화로 해제한 뒤 실행하세요.")
            return
        if not self.robot_connected():
            QtWidgets.QMessageBox.warning(self, "주문 실행", "공정(bed_process)이 연결되어 있지 않습니다.")
            return
        if self.robot.get('running'):
            QtWidgets.QMessageBox.warning(self, "주문 실행", "공정이 진행 중입니다. 끝난 뒤에 실행하세요.")
            return
        self._send('start', mode='order', order={'order_id': order['id'], 'slots': order['slots']})
        self.started_order = order['id']
        self.add_log(f"주문 실행 요청 → [{order['id']}]")

    def _order(self, oid):
        return next((o for o in self.orders if o['id'] == oid), None)

    def _check_started_order(self, d):
        sid = self.started_order
        if sid and d.get('running') and d.get('mode') == 'order':
            self.started_order = None
            o = self._order(sid)
            if o:
                o['status'], o['start'] = '실행 중', datetime.now().strftime("%H:%M:%S")
                self.running_order = sid
                self._save_history()
                self._refresh_order_list()
                self.add_log(f"주문 {sid} 실행 시작")
            return
        rid = self.running_order
        if rid and not d.get('running'):
            self.running_order = None
            o = self._order(rid)
            if o:
                st = d.get('state')
                o['status'] = ('완료 (확인 필요)' if d.get('alarm') else '완료') if st == 'DONE' else '오류' if st == 'FAULT' else '중지'
                o['end'] = datetime.now().strftime("%H:%M:%S")
                o['result'] = d.get('alarm') or ''
                self._save_history()
                self._refresh_order_list()
                self.add_log(f"주문 {rid}: {o['status']}" + (f" ({o['result']})" if o['result'] else ''))


def main(args=None):
    import rclpy
    from threading import Thread
    from cushion_hmi.order_server import create_order_server
    from cushion_hmi.ros_bridge import RosBridge
    try:
        from cushion_hmi.desktop_launcher import ensure_desktop_launcher
    except ImportError:
        ensure_desktop_launcher = None
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    bridge = RosBridge()
    win = Coop1RosHMI(bridge)
    server = server_thread = None
    try:
        if ensure_desktop_launcher is not None:
            try:
                path, created = ensure_desktop_launcher()
                bridge.node.get_logger().info(f'바탕화면 HMI 실행기 {"생성" if created else "이미 있음"}: {path}')
            except (FileExistsError, FileNotFoundError, OSError) as exc:
                bridge.node.get_logger().warning(f'바탕화면 HMI 실행기 설치 실패: {exc}')
        bridge.start()
        win.show()
        try:
            server = create_order_server(bridge.node, bridge.pub_order_config)
        except OSError as exc:
            win.add_log(f"주문 서버 시작 실패 (http://127.0.0.1:5000/order): {exc}")
        else:
            server_thread = Thread(target=server.serve_forever, daemon=True)
            server_thread.start()
            bridge.node.get_logger().info('브라우저 주문 서버 실행: http://127.0.0.1:5000/order')
        return app.exec_()
    finally:
        if server_thread is not None and server_thread.is_alive():
            server.shutdown()
        if server is not None:
            server.server_close()
        bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def demo():
    """팀원 원본 화면 그대로 (ROS 없이 시연)."""
    app = QtWidgets.QApplication(sys.argv)
    win = Coop1MainHMI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    sys.exit(main())
