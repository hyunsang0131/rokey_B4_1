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
try:                                       # test 브랜치의 바탕화면 실행기 (파일이 없으면 건너뜀)
    from cushion_hmi.desktop_launcher import ensure_desktop_launcher
except ImportError:
    ensure_desktop_launcher = None
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
        self.defect = False          # 기본 공정(불량 선별): 말랑 = 불량(빨강), 딱딱 = 정상(파랑)
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
            if self.defect and slot_type in ("S", "H"):
                color = "#d64545" if is_soft else "#2f6fc0"
                fg = "#FFFFFF"
            else:
                color = (
                    "#DFD3C0" if is_soft and height == "HIGH" else
                    "#F4ECE0" if is_soft else
                    "#53687A" if slot_type == "H" else "#eef1f5")
                fg = "#FFFFFF" if slot_type == "H" else "#2C2620"   # 빈칸·미측정은 밝은 바탕이라 진한 글씨
            p.setBrush(QColor(color)); p.setPen(QPen(QColor("#98a9be"),1))
            p.drawRoundedRect(QRectF(x,y,cw,ch),5,5)
            p.setPen(QColor(fg))
            p.setFont(QFont("Sans",10,QFont.Bold))
            text = (
                f"{self.names[i]}\n{slot_type}\n{'불량' if is_soft else '정상'}"
                if self.defect and slot_type in ("S", "H")
                else f"{self.names[i]}\n{slot_type}\n"
                f"{'높은 지지' if height == 'HIGH' else '기본 높이'}"
                if slot_type in ("S", "H")
                else f"{self.names[i]}\n빈칸" if slot_type == "E"
                else f"{self.names[i]}\n미측정" if slot_type == "?"
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

    def set_real(self, curves, types=None):
        """공정에서 받은 실제 힘-변위 곡선 {모듈번호: ([깊이], [힘])} 으로 다시 그린다."""
        self.real = curves
        self.real_types = types or {}
        self.draw_curves()

    def draw_curves(self):
        if self.canvas is None: return
        self.ax.clear()
        colors=["#278bfa","#ed3c3c","#ff9a24","#29a653","#8952e9","#8a4d28"]
        if getattr(self, 'real', None):
            for mid,(xs,ys) in sorted(self.real.items()):
                t = self.real_types.get(mid, '')
                self.ax.plot(xs,ys,label=f"M{mid}" + (f" ({t})" if t else ''),color=colors[(mid-1)%6],linewidth=1.8)
            self.ax.set_xlabel("압입 깊이 z [mm] (접촉점부터)",fontsize=9)
            self.ax.set_ylabel("Fz [N]",fontsize=9)
            self.ax.grid(True,alpha=.22)
            self.ax.legend(fontsize=8,loc="center left",bbox_to_anchor=(1.01,.5),frameon=False)
            self.fig.subplots_adjust(left=.13,right=.75,bottom=.2,top=.96)
            self.canvas.draw_idle()
            return
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
        if getattr(self, 'bridge', None) is not None and self.robot_connected():
            if self.robot.get('running'):
                QMessageBox.warning(self, "제작 실행", "공정이 진행 중입니다. 끝난 뒤에 실행하세요.")
                return
            order = self.orders[index]
            self.curves = {}
            self.bridge.send_cmd('start', mode='order',
                                 order={'order_id': order_id, 'slots': order['slots']})
            self.started_order = order_id            # 공정이 시작한 게 보이면 대기열에서 뺀다
            self.log("제작 실행", f"주문 {order_id} 제작 시작 요청 (주문대로 교체)")
            return
        self.log("제작 실행", f"주문 {order_id}: 공정(bed_process)이 연결되지 않아 보내지 않음")
        QMessageBox.information(
            self,
            "제작 실행",
            "공정(bed_process)이 연결되어 있지 않습니다.\n"
            "bed_all.launch.py 로 공정을 켠 뒤 다시 누르세요.",
        )

    def update_tray_table(self):
        selected=self.tray_select.currentIndex()
        real = getattr(self, 'robot_bed' if selected == 0 else 'robot_tray', None)
        if real:
            names = SLOTS if selected == 0 else TRAY_SLOTS
            kind = {'S': '부드러움', 'H': '단단함', 'T': '부드러움 (높은 지지)', 'E': '빈칸', '?': '미측정'}
            defect = getattr(self, 'defect_mode', False)
            def state(t):
                if t == 'E': return '빈칸'
                if t == '?': return '미측정'
                if defect: return '불량' if t in ('S', 'T') else '정상'
                return '측정 완료'
            rows = [[names[i], kind.get(c['type'], c['type']),
                     '-' if c.get('k') is None else f"{c['k']:.2f}", state(c['type'])] for i, c in enumerate(real)]
            self.detail.setRowCount(len(rows))
            for i,row in enumerate(rows):
                for j,v in enumerate(row):
                    item = QTableWidgetItem(str(v))
                    if j == 3 and v in ('불량', '정상'):
                        item.setForeground(QColor("#c81d25" if v == '불량' else "#1f5fb0"))
                        f = item.font(); f.setBold(True); item.setFont(f)
                    self.detail.setItem(i,j,item)
            return
        if selected==0:
            rows=[[SLOTS[i],INITIAL_TYPES[i],f"{MEASURED_K[i]:.2f}","정상"] for i in range(6)]
        else:
            rows=[[TRAY_SLOTS[i],TRAY_TYPES[i],"미측정","대기"] for i in range(6)]
        self.detail.setRowCount(6)
        for i,row in enumerate(rows):
            for j,v in enumerate(row): self.detail.setItem(i,j,QTableWidgetItem(str(v)))

    STOCK_COUNTS = {'T': 2, 'S': 2, 'H': 6}       # 재고 수량 (2026-11-20 형님): 높은 말랑 2, 일반 말랑 2, 일반 딱딱 6

    def update_stock(self):
        if getattr(self, 'stock_k', None) is not None:
            # 실제 공정 연결 시: 강성은 측정 때마다 그 종류 큐브들의 평균
            k = self.stock_k
            fmt = lambda t: '미측정' if k.get(t) is None else f"{k[t]:.2f} N/mm"      # noqa: E731
            rows=[["높은 말랑 (S, 높은 지지)",fmt('T'),f"{self.STOCK_COUNTS['T']}개","정상"],
                  ["일반 말랑 (S)",fmt('S'),f"{self.STOCK_COUNTS['S']}개","정상"],
                  ["일반 딱딱 (H)",fmt('H'),f"{self.STOCK_COUNTS['H']}개","정상"]]
            for widget in [self.inventory_table,self.stock_table]:
                widget.setRowCount(len(rows))
                for i,row in enumerate(rows):
                    vals=row if widget is self.inventory_table else [row[0],row[2],row[1]]
                    for j,v in enumerate(vals): widget.setItem(i,j,QTableWidgetItem(str(v)))
            return
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

    # ── 실제 공정(bed_process) 연동 (bed_inspect 의 cushion_bridge 가 /cushion/* 로 이어 줌) ──
    MODE_TO_CMD = {"진단 + 교체": "repair", "진단만": "diagnose", "교체만": "order"}

    def attach_bridge(self, bridge):
        self.bridge = bridge
        self.robot = None                # 마지막 /cushion/status
        self.robot_time = 0.0
        self.curves = {}
        bridge.status_received.connect(self.on_robot_status)
        bridge.force_received.connect(self.on_robot_force)
        bridge.log_received.connect(lambda text: self.log("공정", text))
        if hasattr(bridge, 'cmd_result'):
            bridge.cmd_result.connect(self.on_cmd_result)
        try:
            self.stop.clicked.disconnect()
        except TypeError:
            pass
        self.stop.clicked.connect(self.on_stop)

    CMD_KO = {'start': '시작', 'stop': '정지', 'continue': '진행', 'cancel': '취소', 'reset': '초기화',
              'estop': '비상정지', 'estop_reset': '비상정지 해제'}

    def on_cmd_result(self, r):
        """명령 서비스 응답: 받았으면 로그, 거절·응답 없음이면 경고."""
        name = self.CMD_KO.get(r.get('cmd'), r.get('cmd'))
        ok = r.get('accepted')
        if r.get('cmd') == 'estop' and ok:
            self._estop_acked = True
        if r.get('cmd') == 'estop_reset' and ok:
            self.estopped = False
            self.start.setEnabled(True)
        self.log("응답" if ok else "응답 없음" if ok is None else "거절", f"{name}: {r.get('message')}")
        if ok is False:
            if r.get('cmd') == 'start':
                self.start.setEnabled(True)
            box = QMessageBox(QMessageBox.Warning, f"{name} 명령", f"{name} 명령이 처리되지 않았습니다.\n\n{r.get('message')}",
                              QMessageBox.Ok, self)
            box.setModal(False); box.show()
            self._cmd_box = box

    def robot_connected(self):
        import time as _t
        return self.robot is not None and _t.monotonic() - self.robot_time < P.STATUS_TIMEOUT

    def on_stop(self):
        if getattr(self, 'bridge', None) is not None:
            self.bridge.send_cmd('stop')
        self.log("정지", "공정에 정지 요청")

    @staticmethod
    def _diagram_cells(cells):
        # 공정 맵 타입 → 그림: T(키 큰 무른 큐브) = S + 높은 지지
        types = ['S' if c['type'] == 'T' else c['type'] for c in cells]
        heights = ['HIGH' if c['type'] == 'T' else 'NORMAL' for c in cells]
        return types, heights, [c['k'] for c in cells]

    def on_robot_status(self, d):
        import time as _t
        self.robot, self.robot_time = d, _t.monotonic()
        step, name = d.get('step') or 0, d.get('step_name') or ''
        state = P.STATE_LABELS.get(d.get('state'), d.get('state'))
        text = f"● bed_process 연결 · {state}" + (f" · {step}단계 {name}" if d.get('running') else '')
        color = "#c81d25" if d.get('state') == 'FAULT' else "#21824c"
        em = d.get('emergency')
        if em:                                   # 예외 상황: 충돌 긴급 프로토콜 / 재고 부족 큐브 탐색
            text = f"⚠ {em.get('text', '긴급')}" + (f" · {em.get('what')}" if em.get('what') else '')
            color = "#c81d25" if em.get('kind') == 'collision' else "#b35c00"
        self.status.setText(text); self.status.setStyleSheet(f"color:{color};")
        self.show_emergency(em)
        self.show_operator_wait(d)
        self.progress.setValue(100 if d.get('state') == 'DONE' else int(step * 100 / 9))
        self.start.setEnabled(not d.get('running') and not self.estopped)
        bed, tray = d.get('bed_map'), d.get('tray_map')
        mods = {m.get('id'): m for m in d.get('modules') or []}
        if bed:
            (self.mat_diagram.types, self.mat_diagram.heights,
             self.mat_diagram.values) = self._diagram_cells(bed)
        elif mods:
            self.mat_diagram.values = [mods.get(i + 1, {}).get('k') for i in range(6)]
        self.mat_diagram.update()
        if tray:
            (self.stock_diagram.types, self.stock_diagram.heights,
             self.stock_diagram.values) = self._diagram_cells(tray)
            self.stock_diagram.update()
        self.robot_bed, self.robot_tray = bed, tray
        # 제작 실행한 주문이 실제로 시작되면 대기열에서 뺀다 (다음 주문은 다시 '제작 실행' 을 눌러야 시작)
        sid = getattr(self, 'started_order', None)
        if sid and d.get('running') and d.get('mode') == 'order':
            self.started_order = None
            idx = next((i for i, o in enumerate(self.orders) if o['order_id'] == sid), None)
            if idx is not None:
                del self.orders[idx]
                self.selected_order_index = min(self.selected_order_index, len(self.orders) - 1)
                self.update_order_queue()
                self.log("제작 시작", f"주문 {sid} 공정 시작 → 대기열에서 뺌 (남은 주문 {len(self.orders)}건)")
        # 기본 공정(주문 모드가 아닌 것): 말랑 = 불량, 딱딱 = 정상
        self.defect_mode = d.get('mode') not in ('order', None)
        self.mat_diagram.defect = self.stock_diagram.defect = self.defect_mode
        self.update_tray_table()
        self.update_tray_summary(d, bed, tray)
        self.update_stock_k(bed, tray)
        if d.get('alarm') and d.get('alarm') != getattr(self, '_last_alarm', None):
            self._last_alarm = d['alarm']
            self.log("알림", d['alarm'])

    def show_emergency(self, em):
        """예외 상황이 새로 생기면 로그에 남기고 창을 띄운다 (창은 기다리지 않음: 공정은 알아서 복구 후 계속)."""
        key = None if not em else (em.get('kind'), em.get('what'), em.get('retry'))
        if key == getattr(self, '_last_em', None):
            return
        self._last_em = key
        if not em:
            if getattr(self, '_em_box', None) is not None:
                self._em_box.close(); self._em_box = None
                self.log("복구", "예외 상황 처리 끝 → 하던 공정을 이어서 합니다")
            return
        if em.get('kind') == 'collision':
            body = (f"충돌로 인한 긴급 프로토콜\n\n작업: {em.get('what')}\n충돌 위치: {em.get('pos')}\n\n"
                    "정지 → 충돌 반대 방향으로 후퇴 → 홈 → 하던 작업을 다시 합니다.")
        else:
            body = f"{em.get('text')}\n\n{em.get('what') or ''}\n\n탐색 영역에서 큐브를 찾아 트레이에 넣고 이어서 합니다."
        self.log("긴급" if em.get('kind') == 'collision' else "재고 부족", body.replace("\n\n", " / ").replace("\n", " "))
        if getattr(self, '_em_box', None) is not None:
            self._em_box.close()
        box = QMessageBox(QMessageBox.Warning, em.get('text', '긴급'), body, QMessageBox.Ok, self)
        box.setModal(False); box.show()
        self._em_box = box

    def show_operator_wait(self, d):
        """공정이 사람 확인을 기다리면(예: 커버 작업 중 충돌 후) 진행/취소 창을 띄운다."""
        waiting = bool(d.get('wait_operator'))
        if not waiting:
            if getattr(self, '_wait_box', None) is not None:
                self._wait_box.close(); self._wait_box = None
            return
        if getattr(self, '_wait_box', None) is not None:
            return
        box = QMessageBox(QMessageBox.Question, "확인 필요", d.get("prompt") or "진행할까요?",
                          QMessageBox.Ok | QMessageBox.Cancel, self)
        box.button(QMessageBox.Ok).setText("진행"); box.button(QMessageBox.Cancel).setText("취소")
        box.setModal(False)

        def done(r):
            self._wait_box = None
            self.bridge.send_cmd('continue' if r == QMessageBox.Ok else 'cancel')
            self.log("확인", "진행" if r == QMessageBox.Ok else "취소(정지)")
        box.finished.connect(done)
        box.show()
        self._wait_box = box

    def update_tray_summary(self, d, bed, tray):
        def count(cells):
            n = {}
            for c in cells or []:
                n[c['type']] = n.get(c['type'], 0) + 1
            return n
        def text(cells, is_bed):
            if not cells:
                return '미측정'
            n = count(cells)
            soft, hard, empty = n.get('S', 0) + n.get('T', 0), n.get('H', 0), n.get('E', 0)
            if self.defect_mode and is_bed:
                s = f"정상 {hard} · 불량 {soft}"
            else:
                s = f"딱딱 {hard} · 말랑 {soft}" + (f" (높은 {n['T']})" if n.get('T') else '')
            return s + (f" · 빈칸 {empty}" if empty else '')
        step = f"{d.get('step') or 0}단계 {d.get('step_name') or ''}" if d.get('running') else ''
        rows = [["T01", "매트리스 (작업 대상)", "2 × 3 (M1~M6)", (step + ' · ' if step else '') + text(bed, True)],
                ["T02", "재고/예비", "2 × 3 (T2-1~T2-6)", text(tray, False)]]
        for i, row in enumerate(rows):
            for j, v in enumerate(row):
                self.tray_summary.setItem(i, j, QTableWidgetItem(v))

    def update_stock_k(self, bed, tray):
        """재고 현황 강성 = 이번 맵(침대 + 트레이)에서 종류별로 잰 강성의 평균. 이번에 못 잰 종류는 지난 값 유지."""
        if getattr(self, 'stock_k', None) is None:
            self.stock_k = {'T': None, 'S': None, 'H': None}
        for t in ('T', 'S', 'H'):
            ks = [c['k'] for c in (bed or []) + (tray or []) if c['type'] == t and c.get('k')]
            if ks:
                self.stock_k[t] = sum(ks) / len(ks)
        self.update_stock()

    def on_robot_force(self, d):
        try:
            mid, z, fz = int(d['module']), float(d['z']), float(d['fz'])
        except (KeyError, TypeError, ValueError):
            return
        if d.get('reset') or mid not in self.curves:
            self.curves[mid] = ([], [])
        self.curves[mid][0].append(z); self.curves[mid][1].append(fz)
        if not hasattr(self, '_graph_timer'):
            self._graph_timer = QTimer(self); self._graph_timer.setSingleShot(True)
            self._graph_timer.timeout.connect(self._redraw_graph)
        self._graph_timer.start(200)            # 점이 몰려 올 때 한 번만 다시 그린다

    def _redraw_graph(self):
        bed = getattr(self, 'robot_bed', None) or []
        types = {i + 1: c['type'] for i, c in enumerate(bed)}
        self.graph.set_real(self.curves, types)

    def start_demo(self):
        if getattr(self, 'bridge', None) is not None and self.robot_connected():
            mode = self.MODE_TO_CMD.get(self.mode.currentText(), 'repair')
            self.curves = {}
            self.bridge.send_cmd('start', mode=mode)
            self.log("시작", f"{self.mode.currentText()} 공정 시작 요청 ({mode})")
            self.start.setEnabled(False)
            return
        if self.estopped:
            QMessageBox.warning(self,"비상정지 상태","시연 상태가 비상정지로 잠겨 있습니다.")
            return
        self.status.setText("● 시연 중"); self.status.setStyleSheet("color:#21824c;")
        self.log("시작",f"{self.mode.currentText()} 시연 시작")
        self.progress.setValue(25)

    def reset_demo(self):
        if self.estopped:
            if getattr(self, 'bridge', None) is None:
                QMessageBox.warning(self,"비상정지 상태","실제 장비의 안전상태 확인 없이 재가동할 수 없습니다.")
                return
            ans = QMessageBox.question(self, "비상정지 해제",
                                       "로봇 주변 안전을 확인했나요?\n\n해제하면 서보를 다시 켭니다 (로봇은 움직이지 않음).\n"
                                       "공정은 '시작'으로 다시 시작해야 합니다.")
            if ans != QMessageBox.Yes:
                return
            self.bridge.send_cmd('estop_reset')
            self.log("비상정지 해제", "서보 켜기 요청")
            return
        self.progress.setValue(0); self.status.setText("● 미션 노드 대기 중")
        self.log("초기화","UI 상태 초기화")

    def emergency(self):
        self.estopped=True
        self.start.setEnabled(False)
        self.status.setText("● 비상정지")
        self.status.setStyleSheet("color:#c81d25;")
        if getattr(self, 'bridge', None) is None:
            self.log("비상정지","UI 비상정지 버튼 눌림 — ROS 연결 없음")
            return
        # 1) 공정에 비상정지 (공정이 정지 표시를 먼저 세운 뒤 로봇에 빠른 정지 + 서보 끄기 → 충돌 자동 복구가 다시 켜지 않음)
        # 2) 1.5초 안에 받았다는 응답이 없으면 HMI 가 로봇 컨트롤러에 직접 빠른 정지 + 서보 끄기
        self._estop_acked = False
        self.bridge.send_cmd('estop')
        self.log("비상정지", "비상정지 요청 → 공정 (빠른 정지 + 서보 끄기)")
        QTimer.singleShot(1500, self._estop_fallback)
        box = QMessageBox(QMessageBox.Critical, "비상정지",
                          "로봇에 빠른 정지 + 서보 끄기를 보냈습니다.\n\n"
                          "안전을 확인한 뒤 '초기화'를 누르면 비상정지를 해제(서보 켜기)합니다.\n"
                          "※ 이 버튼은 소프트웨어 정지입니다. 위급하면 펜던트/물리 비상정지 버튼을 누르세요.",
                          QMessageBox.Ok, self)
        box.setModal(False); box.show()
        self._estop_box = box

    def _estop_fallback(self):
        if self._estop_acked:
            return
        sent = self.bridge.estop_direct()
        self.log("비상정지", f"공정 응답 없음 → 로봇에 직접: {' + '.join(sent) or '로봇 서비스도 없음 (펜던트 비상정지를 누르세요)'}")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = RosBridge()
    win = MainWindow()
    bridge.order_config_received.connect(win.on_order_config)
    win.attach_bridge(bridge)                 # 실제 공정 상태·그래프·시작/정지·제작 실행 연결 (bed_inspect cushion_bridge)
    server = None
    server_thread = None
    bridge_started = False
    try:
        try:
            if ensure_desktop_launcher is None:
                raise FileNotFoundError('desktop_launcher.py 없음')
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
