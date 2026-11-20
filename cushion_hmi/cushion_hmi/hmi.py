#!/usr/bin/env python3
"""Cushion HMI UI prototype. PyQt5 + matplotlib. No real robot control."""
import math
import random
import sys
from datetime import datetime
from threading import Thread

import rclpy
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QTextEdit, QTabWidget, QMessageBox, QSplitter, QGroupBox,
    QProgressBar, QSizePolicy
)
from cushion_hmi import protocol as P
from cushion_hmi.desktop_launcher import ensure_desktop_launcher
from cushion_hmi.order_server import create_order_server
from cushion_hmi.ros_bridge import RosBridge

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except ImportError:
    FigureCanvas = None

# ===================== USER CONFIGURATION =====================
SOFT_K = 0.80                 # N/mm (DEMO values; replace with measured data)
HARD_K = 1.60                 # N/mm
K_TOLERANCE = 0.15            # N/mm
TRAY_LAYOUT = (P.ROWS, P.COLS)  # rows, columns
SIMULATION_MODE = True        # UI-only. Does not command a real robot.
# ==============================================================
SLOTS = ["M1", "M2", "M3", "M4", "M5", "M6"]
TRAY_SLOTS = [f"T2-{i}" for i in range(1, 7)]
INITIAL_TYPES = ["S", "H", "H", "S", "S", "H"]
TRAY_TYPES = ["S", "S", "H", "S", "H", "H"]
MEASURED_K = [0.80, 1.62, 1.58, 0.82, 0.79, 1.60]

STYLE = """
QMainWindow, QWidget {background:#f5f7fc; color:#1e293b; font-family:'Noto Sans CJK KR','Noto Sans',sans-serif; font-size:12px;}
QGroupBox {background:#f9faff; border:1px solid #cbd4e4; border-radius:10px; margin-top:18px; padding:12px 8px 8px; font-weight:700; font-size:14px;}
QGroupBox::title {subcontrol-origin:margin; left:12px; padding:0 5px;}
QPushButton {background:#f8faff; border:1px solid #c7d1e2; border-radius:8px; padding:9px 13px; font-weight:600;}
QPushButton:hover {background:#e9effa;}
QPushButton:disabled {color:#9ba5b5; background:#f0f2f6;}
QPushButton#estop {background:#dc171c; color:white; border:2px solid #b60c13; font-size:19px; font-weight:900;}
QPushButton#estop:hover {background:#bb0e14;}
QPushButton#selected {background:#405269; color:white; font-size:16px;}
QTabWidget::pane {border:none;}
QTabBar::tab {background:#edf1f9; border:1px solid #cbd4e4; border-radius:8px; padding:13px 22px; margin:2px; font-weight:700; font-size:14px;}
QTabBar::tab:selected {background:#42536a; color:white;}
QTableWidget {background:#fff; border:1px solid #d7deeb; gridline-color:#e1e7f0; border-radius:6px;}
QHeaderView::section {background:#e8edf6; padding:7px; border:0; font-weight:700;}
QTextEdit {background:white; border:1px solid #d1d9e7; border-radius:6px;}
QComboBox {background:white; border:1px solid #cbd4e4; border-radius:6px; padding:6px;}
QProgressBar {background:#e6ebf3; border:0; border-radius:5px; text-align:center;}
QProgressBar::chunk {background:#56aa78; border-radius:5px;}
"""

def label(text, size=12, bold=False, color=None):
    w = QLabel(text)
    f = w.font(); f.setPointSize(size); f.setBold(bold); w.setFont(f)
    if color: w.setStyleSheet(f"color:{color};")
    return w

def table(headers, rows):
    t = QTableWidget(len(rows), len(headers))
    t.setHorizontalHeaderLabels(headers)
    t.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    t.verticalHeader().hide()
    t.setEditTriggers(QTableWidget.NoEditTriggers)
    t.setSelectionBehavior(QTableWidget.SelectRows)
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            item = QTableWidgetItem(str(val))
            item.setTextAlignment(Qt.AlignCenter if j < 3 else Qt.AlignVCenter)
            t.setItem(i, j, item)
    return t


def random_shipping_address():
    """Return a visibly fictional sample address for the UI demo."""
    city = random.choice(("서울특별시", "부산광역시", "대전광역시", "광주광역시"))
    district = random.choice(("강남구", "해운대구", "유성구", "북구"))
    road = random.choice(("쿠션로", "편안길", "매트리스대로", "포근길"))
    number = random.randint(10, 248)
    unit = random.randint(101, 1804)
    return random.choice((
        f"샘플 주소 · {city} {district} {road} {number}길 {unit}호",
        f"샘플 주소 · {city} {district} {road} {number}, {unit}동",
        f"샘플 주소 · {city} {district} {road} {number}-{unit}",
    ))


class TrayDiagram(QWidget):
    """Stylized schematic, NOT a camera image."""
    def __init__(self, types, names, title, parent=None):
        super().__init__(parent)
        self.types = types; self.names = names; self.title = title
        self.heights = ['NORMAL'] * 6
        self.values = [None]*6
        self.setMinimumHeight(180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(10, 10, -10, -10)
        p.setPen(QPen(QColor("#aeb8c9"), 1))
        p.setBrush(QColor("#e7d9c6")); p.drawRoundedRect(QRectF(r), 12, 12)
        title_h = 27
        p.setPen(QColor("#263347")); p.setFont(QFont("Sans", 10, QFont.Bold))
        p.drawText(r.left()+13, r.top()+20, self.title)
        x0, y0 = r.left()+r.width()*.19, r.top()+title_h+5
        w, h = r.width()*.72, r.height()-title_h-18
        p.setBrush(QColor("#202a35")); p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(x0-9,y0-9,w+18,h+18), 9,9)
        gap = 6
        rows, columns = TRAY_LAYOUT
        cw = (w - (columns - 1) * gap) / columns
        ch = (h - (rows - 1) * gap) / rows
        for i in range(6):
            row, col = divmod(i, columns)
            x = x0 + col * (cw + gap)
            y = y0 + row * (ch + gap)
            slot_type = self.types[i]
            height = self.heights[i]
            is_soft = slot_type == "S"
            color = (
                "#DFD3C0" if is_soft and height == "HIGH" else
                "#F4ECE0" if is_soft else
                "#53687A" if slot_type == "H" else "#eef1f5")
            p.setBrush(QColor(color)); p.setPen(QPen(QColor("#98a9be"),1))
            p.drawRoundedRect(QRectF(x,y,cw,ch),5,5)
            p.setPen(QColor("#2C2620" if is_soft else "#FFFFFF"))
            p.setFont(QFont("Sans",10,QFont.Bold))
            text = (
                f"{self.names[i]}\n{slot_type}\n"
                f"{'높은 지지' if height == 'HIGH' else '기본 높이'}"
                if slot_type in ("S", "H")
                else f"{self.names[i]}\n주문 대기")
            if self.values[i] is not None: text+=f"\n({self.values[i]:.2f})"
            p.drawText(QRectF(x,y,cw,ch),Qt.AlignCenter,text)
        p.end()

    def set_order(self, slots):
        names = [f"Slot {index}" for index in range(1, 7)]
        self.types = [
            "S" if slots[name]["stiffness"] == "부드러움" else "H"
            for name in names]
        self.heights = [slots[name]["height"] for name in names]
        self.values = [
            P.STIFFNESS_K[slots[name]["stiffness"]] for name in names]
        self.update()


class Graph(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay=QVBoxLayout(self); lay.setContentsMargins(0,0,0,0)
        if FigureCanvas is None:
            lay.addWidget(label("matplotlib 설치 후 힘–변위 그래프 표시",12))
            self.canvas=None; return
        self.fig=Figure(figsize=(5,3),dpi=95,facecolor="#f9faff")
        self.ax=self.fig.add_subplot(111)
        self.canvas=FigureCanvas(self.fig)
        lay.addWidget(self.canvas)
        self.draw_curves()

    def draw_curves(self):
        if self.canvas is None: return
        self.ax.clear()
        colors=["#278bfa","#ed3c3c","#ff9a24","#29a653","#8952e9","#8a4d28"]
        for i,(name,k) in enumerate(zip(SLOTS,MEASURED_K)):
            xs=[j*.1 for j in range(31)]
            ys=[k*x*(1+.04*x) * 18 for x in xs]  # Demo only; arbitrary display scaling
            self.ax.plot(xs,ys,label=f"{name} ({INITIAL_TYPES[i]})",color=colors[i],linewidth=1.8)
        self.ax.set_xlabel("압입 깊이 z [mm]",fontsize=9)
        self.ax.set_ylabel("Fz [N] (시연용)",fontsize=9)
        self.ax.grid(True,alpha=.22)
        self.ax.legend(fontsize=8,loc="center left",bbox_to_anchor=(1.01,.5),frameon=False)
        self.fig.subplots_adjust(left=.13,right=.75,bottom=.2,top=.96)
        self.canvas.draw_idle()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("협동1 쿠션 진단 HMI — UI 프로토타입")
        self.resize(1360,780)
        self.setStyleSheet(STYLE)
        self.estopped=False
        self.stock={"S":24,"H":18}
        self.logs=[]
        self.orders = []
        self.selected_order_index = -1
        self.build()
        self.log("시스템","HMI UI 시연 모드 시작 — ROS 2 / 로봇 연결 없음")

    def build(self):
        root=QWidget(); self.setCentralWidget(root)
        outer=QVBoxLayout(root); outer.setContentsMargins(12,10,12,12); outer.setSpacing(8)
        header=QHBoxLayout()
        header.addWidget(label("매트리스 쿠션감 진단 · 모듈 교체",18,True))
        self.status=label("● 미션 노드 대기 중",12,True,"#56657f")
        header.addWidget(self.status); header.addStretch()
        header.addWidget(label("모드"))
        self.mode=QComboBox(); self.mode.addItems(["진단 + 교체","진단만","교체만"]); header.addWidget(self.mode)
        self.start=QPushButton("▶ 시작"); self.start.clicked.connect(self.start_demo); header.addWidget(self.start)
        self.stop=QPushButton("정지 요청"); self.stop.clicked.connect(lambda:self.log("정지","정지 요청 (시연용)")); header.addWidget(self.stop)
        self.reset=QPushButton("초기화"); self.reset.clicked.connect(self.reset_demo); header.addWidget(self.reset)
        self.estop=QPushButton("⚠\n비상 정지"); self.estop.setObjectName("estop")
        self.estop.setMinimumSize(155,90); self.estop.clicked.connect(self.emergency)
        header.addWidget(self.estop)
        outer.addLayout(header)
        self.tabs=QTabWidget()
        self.tabs.addTab(self.inventory_tab(),"▣  재고 관리")
        self.tabs.addTab(self.tray_tab(),"▦  트레이 현황")
        self.tabs.addTab(self.rviz_tab(),"♧  작업 상태 (RViz)")
        self.tabs.addTab(self.order_tab(),"▤  고객 주문 사항")
        self.tabs.setCurrentIndex(1)
        outer.addWidget(self.tabs,1)

    def group(self,title,child):
        box=QGroupBox(title); lay=QVBoxLayout(box); lay.addWidget(child); return box

    def tray_tab(self):
        page=QWidget(); lay=QVBoxLayout(page); lay.setSpacing(8)
        top=QHBoxLayout(); top.setSpacing(8)
        left=QWidget(); ll=QVBoxLayout(left); ll.setContentsMargins(0,0,0,0)
        self.mat_diagram=TrayDiagram(INITIAL_TYPES,SLOTS,"트레이 T01 · 매트리스")
        self.mat_diagram.values=MEASURED_K
        self.stock_diagram=TrayDiagram(TRAY_TYPES,TRAY_SLOTS,"트레이 T02 · 재고/예비")
        ll.addWidget(self.group("작업 환경 현황 (모식도)",self.mat_diagram),3)
        ll.addWidget(self.group("재고 트레이 (2 × 3)",self.stock_diagram),2)
        top.addWidget(left,4)
        middle=QWidget(); ml=QVBoxLayout(middle); ml.setContentsMargins(0,0,0,0)
        self.tray_select=QComboBox(); self.tray_select.addItems(["T01 (매트리스)","T02 (재고/예비)"])
        self.tray_select.currentIndexChanged.connect(self.update_tray_table)
        midgroup=QGroupBox("트레이 상세 정보")
        midlay=QVBoxLayout(midgroup); midlay.addWidget(self.tray_select)
        self.detail=table(["슬롯","블록 타입","강성 k (N/mm)","측정 상태"],[])
        midlay.addWidget(self.detail); ml.addWidget(midgroup,3)
        self.tray_summary=table(["트레이","용도","슬롯 구성","상태"],[
            ["T01","매트리스 (작업 대상)","2 × 3 (M1~M6)","측정 완료"],
            ["T02","재고/예비","2 × 3 (T2-1~T2-6)","대기"]])
        ml.addWidget(self.group("트레이 상태 요약",self.tray_summary),2)
        top.addWidget(middle,4)
        right=QWidget(); rl=QVBoxLayout(right); rl.setContentsMargins(0,0,0,0)
        self.graph=Graph(); rl.addWidget(self.group("힘–변위 그래프 (T01)",self.graph),3)
        self.stock_table=table(["블록 타입","재고 수량","기준 강성 k"],[])
        rl.addWidget(self.group("실리콘 블록 재고 현황",self.stock_table),2)
        top.addWidget(right,4)
        lay.addLayout(top,5)
        self.log_table=table(["시간","대상","이벤트","상세 내용"],[])
        lay.addWidget(self.group("작업 로그 (트레이 / 측정 / 교체 이력)",self.log_table),2)
        self.update_tray_table(); self.update_stock()
        return page

    def inventory_tab(self):
        page=QWidget(); lay=QVBoxLayout(page)
        lay.addWidget(label("실리콘 블록 재고 관리",18,True))
        info=label("실측 강성값, 위치별 블록 상태, 교체 이력을 관리합니다. 아래 버튼은 시연용입니다.",12)
        lay.addWidget(info)
        self.inventory_table=table(["블록 타입","기준 강성","가용 수량","상태"],[])
        lay.addWidget(self.group("재고 현황",self.inventory_table),2)
        row=QHBoxLayout()
        self.locate=QPushButton("⌖ 바닥 블록 위치 추정 실행 (시연)")
        self.locate.clicked.connect(lambda:self.log("위치 추정","바닥 블록 탐색 요청 (시연용, 로봇 동작 없음)"))
        row.addWidget(self.locate); row.addStretch()
        lay.addLayout(row)
        self.inventory_history=QTextEdit(); self.inventory_history.setReadOnly(True)
        lay.addWidget(self.group("재고 변경 이력",self.inventory_history),2)
        return page

    def rviz_tab(self):
        page=QWidget(); lay=QVBoxLayout(page)
        lay.addWidget(label("작업 상태 · RViz 연동 영역",18,True))
        note=QLabel("RViz2 시각화 자리표시자\n\n실제 연동 시 RViz2 별도 창 실행 또는\nQt 기반 RViz 렌더 패널 통합 필요\n\n로봇 관절 상태: 미연결\n그리퍼 상태: 미연결\n현재 공정: 대기")
        note.setAlignment(Qt.AlignCenter); note.setStyleSheet("background:#e8edf6;border:1px dashed #9aa9be;border-radius:12px;font-size:19px;color:#56657f;")
        lay.addWidget(note,1)
        self.progress=QProgressBar(); self.progress.setValue(0); lay.addWidget(self.progress)
        return page

    def order_tab(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.addWidget(label("고객 주문 사항",18,True))

        content = QHBoxLayout()
        queue_panel = QWidget()
        queue_layout = QVBoxLayout(queue_panel)
        queue_layout.setContentsMargins(0, 0, 0, 0)
        self.order_queue_count = label("접수 주문 0건", 13, True)
        queue_layout.addWidget(self.order_queue_count)
        self.order_queue = QTableWidget(0, 3)
        self.order_queue.setHorizontalHeaderLabels(
            ["순서", "주문 번호", "모델"])
        self.order_queue.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.order_queue.verticalHeader().hide()
        self.order_queue.setEditTriggers(QTableWidget.NoEditTriggers)
        self.order_queue.setSelectionBehavior(QTableWidget.SelectRows)
        self.order_queue.setSelectionMode(QTableWidget.SingleSelection)
        self.order_queue.currentCellChanged.connect(
            self.on_order_selection_changed)
        queue_layout.addWidget(self.order_queue, 1)

        actions = QGridLayout()
        self.delete_order_button = QPushButton("주문 삭제")
        self.move_order_up_button = QPushButton("우선순위 위로")
        self.move_order_down_button = QPushButton("우선순위 아래로")
        self.execute_order_button = QPushButton("제작 실행")
        self.delete_order_button.clicked.connect(self.delete_selected_order)
        self.move_order_up_button.clicked.connect(
            self.move_selected_order_up)
        self.move_order_down_button.clicked.connect(
            self.move_selected_order_down)
        self.execute_order_button.clicked.connect(
            self.execute_selected_order)
        actions.addWidget(self.delete_order_button, 0, 0)
        actions.addWidget(self.move_order_up_button, 0, 1)
        actions.addWidget(self.move_order_down_button, 1, 0)
        actions.addWidget(self.execute_order_button, 1, 1)
        queue_layout.addLayout(actions)
        content.addWidget(self.group("주문 대기열", queue_panel), 2)

        selected_order_panel = QWidget()
        selected_layout = QHBoxLayout(selected_order_panel)
        selected_layout.setContentsMargins(0, 0, 0, 0)
        diagram_panel = QWidget()
        diagram_layout = QVBoxLayout(diagram_panel)
        diagram_layout.setContentsMargins(0, 0, 0, 0)
        self.order_diagram = TrayDiagram(
            ["-"] * 6, SLOTS, "HTML 주문 작업 현황")
        self.order_diagram.values = [None] * 6
        diagram_layout.addWidget(
            self.group("주문 구성 이미지", self.order_diagram), 1)
        diagram_layout.addWidget(label(
            "파란색: 딱딱함 · 밝은 베이지: 부드러움/기본 높이 · "
            "진한 베이지: 부드러움/높은 지지", 10))
        selected_layout.addWidget(diagram_panel, 3)

        details = QWidget()
        details_layout = QVBoxLayout(details)
        self.order_number = label("주문 번호\nHTML 주문 대기 중",16,True)
        self.order_number.setWordWrap(True)
        details_layout.addWidget(self.group("주문 정보", self.order_number), 1)

        self.order_address = QLabel(
            "임시 배송 주소\n주문을 수신하면 샘플 주소를 생성합니다.")
        self.order_address.setWordWrap(True)
        self.order_address.setStyleSheet(
            "background:white;border:1px solid #d1d9e7;border-radius:6px;padding:14px;")
        details_layout.addWidget(self.group("배송 주소 (시연용)", self.order_address), 1)

        self.order_summary = QLabel(
            "여섯 슬롯의 강성 선택값이 주문 이미지에 반영됩니다.")
        self.order_summary.setWordWrap(True)
        details_layout.addWidget(self.group("슬롯 구성", self.order_summary), 2)
        details_layout.addStretch(1)
        selected_layout.addWidget(details, 2)
        content.addWidget(
            self.group("선택 주문 상세", selected_order_panel), 5)
        lay.addLayout(content, 1)
        self.update_order_queue()
        return page

    def on_order_config(self, data):
        try:
            slots = P.normalize_order_slots(data.get("slots"))
        except ValueError as exc:
            self.log("주문 거부", str(exc))
            return

        order_id = data.get("order_id")
        if not isinstance(order_id, str) or not order_id or len(order_id) > 64:
            self.log("주문 거부", "주문 번호가 없거나 올바르지 않습니다")
            return

        bed_model = data.get("bed_model", "모델 정보 없음")
        size = data.get("size", "")
        created_at = data.get("created_at")
        if (not isinstance(bed_model, str) or len(bed_model) > 64
                or not isinstance(size, str) or len(size) > 32):
            self.log("주문 거부", "주문 모델 또는 규격 정보가 올바르지 않습니다")
            return
        if created_at is not None and (
                not isinstance(created_at, str) or len(created_at) > 64):
            self.log("주문 거부", "주문 접수 시간이 올바르지 않습니다")
            return

        order = {
            "order_id": order_id,
            "bed_model": bed_model,
            "size": size,
            "created_at": created_at or datetime.now().isoformat(
                timespec="seconds"),
            "slots": dict(slots),
            "address": random_shipping_address(),
        }
        self.orders.append(order)
        self.selected_order_index = len(self.orders) - 1
        self.update_order_queue()
        self.log(
            "주문 수신",
            f"HTML 주문 {order_id}을 대기열 {len(self.orders)}번에 추가")

    def update_order_queue(self):
        if not hasattr(self, "order_queue"):
            return

        self.order_queue.blockSignals(True)
        self.order_queue.setRowCount(len(self.orders))
        for index, order in enumerate(self.orders):
            values = (
                str(index + 1),
                order["order_id"],
                f'{order["bed_model"]} {order["size"]}'.strip(),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.order_queue.setItem(index, column, item)
        self.order_queue_count.setText(f"접수 주문 {len(self.orders)}건")
        if 0 <= self.selected_order_index < len(self.orders):
            self.order_queue.setCurrentCell(self.selected_order_index, 0)
        else:
            self.selected_order_index = -1
            self.order_queue.clearSelection()
        self.order_queue.blockSignals(False)
        self.show_selected_order()

    def on_order_selection_changed(self, current_row, *_args):
        self.selected_order_index = current_row
        self.show_selected_order()

    def show_selected_order(self):
        index = self.selected_order_index
        has_selection = 0 <= index < len(self.orders)
        self.delete_order_button.setEnabled(has_selection)
        self.execute_order_button.setEnabled(has_selection)
        self.move_order_up_button.setEnabled(has_selection and index > 0)
        self.move_order_down_button.setEnabled(
            has_selection and index < len(self.orders) - 1)
        if not has_selection:
            self.order_diagram.types = ["-"] * 6
            self.order_diagram.heights = ["NORMAL"] * 6
            self.order_diagram.values = [None] * 6
            self.order_diagram.update()
            self.order_number.setText("주문 번호\n주문을 선택하세요")
            self.order_address.setText("주문을 선택하면 배송 정보가 표시됩니다.")
            self.order_summary.setText("선택한 주문의 슬롯 구성이 표시됩니다.")
            return

        order = self.orders[index]
        slots = order["slots"]
        self.order_diagram.set_order(slots)
        self.order_number.setText(
            f'주문 번호\n{order["order_id"]}\n'
            f'{order["bed_model"]} {order["size"]}'.strip())
        self.order_address.setText(f'샘플 배송 주소\n{order["address"]}')
        self.order_summary.setText("\n".join(
            f"M{slot}: {self.format_order_slot(slots[f'Slot {slot}'])}"
            for slot in range(1, 7)))

    @staticmethod
    def format_order_slot(slot):
        stiffness = slot["stiffness"]
        height = "높은 지지" if slot["height"] == "HIGH" else "기본 높이"
        return (
            f"{stiffness} ({P.STIFFNESS_K[stiffness]:g} N/mm) · {height}")

    def delete_selected_order(self):
        index = self.selected_order_index
        if not 0 <= index < len(self.orders):
            return
        order_id = self.orders[index]["order_id"]
        del self.orders[index]
        self.selected_order_index = min(index, len(self.orders) - 1)
        self.update_order_queue()
        self.log("주문 삭제", f"주문 {order_id}을 대기열에서 삭제")

    def move_selected_order_up(self):
        index = self.selected_order_index
        if not 0 < index < len(self.orders):
            return
        self.orders[index - 1], self.orders[index] = (
            self.orders[index], self.orders[index - 1])
        self.selected_order_index = index - 1
        self.update_order_queue()
        self.log(
            "우선순위 변경",
            f"주문 {self.orders[index - 1]['order_id']}을 한 단계 올림")

    def move_selected_order_down(self):
        index = self.selected_order_index
        if not 0 <= index < len(self.orders) - 1:
            return
        self.orders[index], self.orders[index + 1] = (
            self.orders[index + 1], self.orders[index])
        self.selected_order_index = index + 1
        self.update_order_queue()
        self.log(
            "우선순위 변경",
            f"주문 {self.orders[index + 1]['order_id']}을 한 단계 내림")

    def execute_selected_order(self):
        index = self.selected_order_index
        if not 0 <= index < len(self.orders):
            return
        order_id = self.orders[index]["order_id"]
        self.log("제작 실행", f"주문 {order_id} 제작 실행 버튼 선택 (ROS 미연동)")
        QMessageBox.information(
            self,
            "제작 실행",
            "이 버튼은 아직 화면 동작만 준비되어 있습니다.\n"
            "ROS 2 제작 명령은 전송되지 않았습니다.",
        )

    def update_tray_table(self):
        selected=self.tray_select.currentIndex()
        if selected==0:
            rows=[[SLOTS[i],INITIAL_TYPES[i],f"{MEASURED_K[i]:.2f}","정상"] for i in range(6)]
        else:
            rows=[[TRAY_SLOTS[i],TRAY_TYPES[i],"미측정","대기"] for i in range(6)]
        self.detail.setRowCount(6)
        for i,row in enumerate(rows):
            for j,v in enumerate(row): self.detail.setItem(i,j,QTableWidgetItem(str(v)))

    def update_stock(self):
        rows=[["S (부드러움)",f"{SOFT_K:.2f} N/mm",f"{self.stock['S']}개","정상"],
              ["H (단단함)",f"{HARD_K:.2f} N/mm",f"{self.stock['H']}개","정상"]]
        for widget in [self.inventory_table,self.stock_table]:
            widget.setRowCount(2)
            for i,row in enumerate(rows):
                vals=row if widget is self.inventory_table else [row[0],row[2],row[1]]
                for j,v in enumerate(vals): widget.setItem(i,j,QTableWidgetItem(str(v)))

    def log(self,kind,detail,target="T01"):
        tm=datetime.now().strftime("%H:%M:%S")
        self.logs.insert(0,[tm,target,kind,detail])
        if hasattr(self,"log_table"):
            self.log_table.setRowCount(min(100,len(self.logs)))
            for i,row in enumerate(self.logs[:100]):
                for j,v in enumerate(row): self.log_table.setItem(i,j,QTableWidgetItem(v))
        if hasattr(self,"inventory_history"):
            self.inventory_history.append(f"[{tm}] [{kind}] {detail}")

    def start_demo(self):
        if self.estopped:
            QMessageBox.warning(self,"비상정지 상태","시연 상태가 비상정지로 잠겨 있습니다.")
            return
        self.status.setText("● 시연 중"); self.status.setStyleSheet("color:#21824c;")
        self.log("시작",f"{self.mode.currentText()} 시연 시작")
        self.progress.setValue(25)

    def reset_demo(self):
        if self.estopped:
            QMessageBox.warning(self,"비상정지 상태","실제 장비의 안전상태 확인 없이 재가동할 수 없습니다.")
            return
        self.progress.setValue(0); self.status.setText("● 미션 노드 대기 중")
        self.log("초기화","UI 상태 초기화")

    def emergency(self):
        self.estopped=True
        self.start.setEnabled(False); self.reset.setEnabled(False)
        self.status.setText("● 비상정지 요청 (UI 시연)")
        self.status.setStyleSheet("color:#c81d25;")
        self.log("비상정지","UI 비상정지 버튼 눌림 — 실제 안전회로와 미연결")
        QMessageBox.critical(self,"비상정지 (시연)","UI 동작이 잠겼습니다.\n\n주의: 이 버튼은 실제 로봇 비상정지 회로에 연결되지 않았습니다.\n실제 장비는 인증된 물리적 E-STOP으로 정지해야 합니다.")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = RosBridge()
    win = MainWindow()
    bridge.order_config_received.connect(win.on_order_config)
    server = None
    server_thread = None
    bridge_started = False
    try:
        try:
            launcher_path, created = ensure_desktop_launcher()
        except (FileExistsError, FileNotFoundError, OSError) as exc:
            bridge.node.get_logger().warning(
                f'바탕화면 HMI 실행기 설치 실패: {exc}')
        else:
            action = '생성' if created else '이미 있음'
            bridge.node.get_logger().info(
                f'바탕화면 HMI 실행기 {action}: {launcher_path}')
        bridge.start()
        bridge_started = True
        win.show()
        try:
            server = create_order_server(
                bridge.node, bridge.pub_order_config)
        except OSError as exc:
            message = (
                f'주문 서버를 시작하지 못했습니다 '
                f'(http://127.0.0.1:5000/order): {exc}')
            bridge.node.get_logger().error(message)
            win.status.setText('● 주문 서버 시작 실패')
            win.status.setStyleSheet('color:#c81d25;')
            win.log('주문 서버 오류', message)
        else:
            server_thread = Thread(
                target=server.serve_forever, daemon=True)
            server_thread.start()
            bridge.node.get_logger().info(
                '브라우저 주문 서버 실행: http://127.0.0.1:5000/order')
        return app.exec_()
    finally:
        if server_thread is not None and server_thread.is_alive():
            server.shutdown()
        if server is not None:
            server.server_close()
        if bridge_started:
            bridge.shutdown()
        else:
            bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
