# HMI 화면 부품: 모듈 격자, 상태 표시줄, 힘-변위 그래프

import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from cushion_hmi import protocol as P

# 모듈 상태별 배경색, 테두리색
STATUS_COLORS = {
    'unknown': ('#eef0f3', '#c9ced6'),
    'measuring': ('#dbeafe', '#3b82f6'),
    'measured': ('#e0f2fe', '#7aa7c7'),
    'ok': ('#dcfce7', '#22a05a'),
    'defect': ('#fee2e2', '#dc2626'),
    'replaced': ('#ede9fe', '#7c3aed'),
}
ACCENT = '#1f5f8b'


def fmt(v, digits=2):
    return '—' if v is None else f'{v:.{digits}f}'


class ModuleCell(QFrame):
    def __init__(self, mid):
        super().__init__()
        self.mid = mid
        self.setObjectName('cell')
        self.setMinimumSize(150, 110)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(2)
        self.title = QLabel(f'M{mid}')
        self.title.setFont(QFont(self.font().family(), 15, QFont.Bold))
        self.status = QLabel()
        self.values = QLabel()
        self.values.setStyleSheet('font-family: monospace; color: #374151;')
        lay.addWidget(self.title)
        lay.addWidget(self.status)
        lay.addStretch(1)
        lay.addWidget(self.values)
        self.set_data({}, False)

    def set_data(self, d, is_current):
        st = d.get('status', 'unknown')
        bg, border = STATUS_COLORS.get(st, STATUS_COLORS['unknown'])
        width = 3 if is_current else 1
        self.setStyleSheet(
            f'#cell {{ background: {bg}; border: {width}px solid {border}; border-radius: 8px; }}')
        name = d.get('name')
        self.title.setText(f'M{self.mid}' + (f'  ·  {name}' if name else ''))
        self.status.setText(P.MODULE_STATUS.get(st, st))
        self.status.setStyleSheet(f'color: {border}; font-weight: bold;')
        # 정지 측정(fz)을 보내면 Fz 평균·표준편차, 아니면 압입 측정 결과(k, z0)
        if 'fz' in d:
            self.values.setText(f'Fz {fmt(d.get("fz"))} N\n ± {fmt(d.get("fz_std"))} N')
        else:
            self.values.setText(f'k  {fmt(d.get("k"))} N/mm\nz0 {fmt(d.get("z0"))} mm')


class ModuleGrid(QWidget):
    """가로 2 x 세로 3 모듈 배치. 위쪽 = 선반, 왼쪽 = 로봇 (배치 v3)."""

    def __init__(self):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        top = QLabel('▲ 선반 쪽')
        top.setAlignment(Qt.AlignCenter)
        top.setStyleSheet('color: #6b7280;')
        outer.addWidget(top)

        row = QHBoxLayout()
        side = QLabel('◀\n로\n봇')
        side.setAlignment(Qt.AlignCenter)
        side.setStyleSheet('color: #6b7280;')
        row.addWidget(side)
        grid = QGridLayout()
        grid.setSpacing(10)
        self.cells = {}
        for mid in P.module_ids():
            r, c = P.module_rc(mid)
            cell = ModuleCell(mid)
            self.cells[mid] = cell
            grid.addWidget(cell, r, c)
        row.addLayout(grid, 1)
        outer.addLayout(row, 1)

        legend = QHBoxLayout()
        legend.setSpacing(12)
        for key in ('unknown', 'measuring', 'ok', 'defect', 'replaced'):
            bg, border = STATUS_COLORS[key]
            sw = QLabel()
            sw.setFixedSize(12, 12)
            sw.setStyleSheet(f'background: {bg}; border: 1px solid {border}; border-radius: 2px;')
            legend.addWidget(sw)
            legend.addWidget(QLabel(P.MODULE_STATUS[key]))
        legend.addStretch(1)
        outer.addLayout(legend)

    def set_modules(self, modules, current):
        by_id = {m.get('id'): m for m in modules if isinstance(m, dict)}
        for mid, cell in self.cells.items():
            cell.set_data(by_id.get(mid, {}), mid == current)


class StateBar(QWidget):
    """플로우차트 단계를 한 줄로 보여주고 현재 단계를 강조한다."""

    def __init__(self):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        self.labels = {}
        for key, text in P.FLOW_STATES:
            lab = QLabel(text)
            lab.setAlignment(Qt.AlignCenter)
            lab.setMinimumHeight(30)
            self.labels[key] = lab
            lay.addWidget(lab, 1)
        self.extra = QLabel()
        self.extra.setAlignment(Qt.AlignCenter)
        self.extra.setMinimumWidth(70)
        lay.addWidget(self.extra)
        self.set_state('IDLE')

    def set_state(self, state):
        for key, lab in self.labels.items():
            if key == state:
                lab.setStyleSheet(f'background: {ACCENT}; color: white; font-weight: bold;'
                                  'border-radius: 4px;')
            else:
                lab.setStyleSheet('background: #eef0f3; color: #4b5563; border-radius: 4px;')
        if state in self.labels:
            self.extra.setText('')
            self.extra.setStyleSheet('')
            return
        colors = {'FAULT': '#dc2626', 'DONE': '#22a05a'}
        c = colors.get(state, '#6b7280')
        self.extra.setText(P.STATE_LABELS.get(state, state or '—'))
        self.extra.setStyleSheet(f'color: white; background: {c}; font-weight: bold;'
                                 'border-radius: 4px; padding: 4px 8px;')


def nice_step(span):
    """눈금 간격: span/5 이상인 1, 2, 2.5, 5 x 10^n."""
    raw = max(span, 1e-9) / 5
    e = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * e + 1e-12:
            return m * e
    return 10 * e


def nice_range(lo, hi, min_span=1.0):
    """0 을 포함하고 눈금이 깔끔하게 떨어지는 축 범위 (lo, hi, step)."""
    lo, hi = min(lo, 0.0), max(hi, 0.0)
    # 0 근처 노이즈(예: 접촉 직후 -0.08 N) 때문에 축이 크게 내려가지 않게 한다
    if lo < 0 < hi and -lo < 0.05 * hi:
        lo = 0.0
    if hi > 0 > lo and hi < 0.05 * -lo:
        hi = 0.0
    if hi - lo < min_span:
        hi = lo + min_span
    step = nice_step(hi - lo)
    lo = math.floor(lo / step + 1e-9) * step
    hi = math.ceil(hi / step - 1e-9) * step
    return lo, hi, step


X_LABELS = {'depth': '압입 깊이 z [mm]', 'time': '측정 시간 t [s]'}


class ForcePlot(QWidget):
    """힘 그래프. 가로축은 압입 깊이(depth) 또는 시간(time).
    모듈마다 마지막 측정 곡선을 남기고, 현재 모듈을 강조한다."""

    MARGIN_L, MARGIN_R, MARGIN_T, MARGIN_B = 52, 28, 14, 40

    def __init__(self):
        super().__init__()
        self.setMinimumSize(360, 260)
        self.traces = {}
        self.current = None
        self.kind = 'depth'

    def clear(self):
        self.traces = {}
        self.current = None
        self.update()

    def add_point(self, mid, x, fz, reset, kind='depth'):
        if kind != self.kind:       # 가로축 종류가 바뀌면 이전 곡선은 비교할 수 없으므로 지운다
            self.kind = kind
            self.traces = {}
        if reset or mid not in self.traces:
            self.traces[mid] = []
        self.traces[mid].append((x, fz))
        self.current = mid
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#ffffff'))
        plot = QRectF(self.MARGIN_L, self.MARGIN_T,
                      self.width() - self.MARGIN_L - self.MARGIN_R,
                      self.height() - self.MARGIN_T - self.MARGIN_B)

        pts = [pt for tr in self.traces.values() for pt in tr]
        xs = [x for x, _ in pts] or [0.0]
        fs = [f for _, f in pts] or [0.0]
        x0, x1, xstep = nice_range(min(xs), max(xs))
        # 정지 측정은 값이 작고 변화가 적어서, 0 대신 데이터 범위에 맞춘다
        if self.kind == 'time' and pts:
            pad = max(0.5, (max(fs) - min(fs)) * 0.2)
            ystep = nice_step(max(fs) - min(fs) + 2 * pad)
            y0 = math.floor((min(fs) - pad) / ystep) * ystep
            y1 = math.ceil((max(fs) + pad) / ystep) * ystep
        else:
            y0, y1, ystep = nice_range(min(fs), max(fs))

        def to_px(x, f):
            px = plot.left() + plot.width() * (x - x0) / (x1 - x0)
            py = plot.bottom() - plot.height() * (f - y0) / (y1 - y0)
            return QPointF(px, py)

        # 격자와 눈금
        p.setFont(QFont(self.font().family(), 8))
        for i in range(int(round((x1 - x0) / xstep)) + 1):
            v = x0 + i * xstep
            px = to_px(v, y0).x()
            p.setPen(QPen(QColor('#e5e7eb'), 1))
            p.drawLine(QPointF(px, plot.top()), QPointF(px, plot.bottom()))
            p.setPen(QColor('#6b7280'))
            p.drawText(QRectF(px - 24, plot.bottom() + 4, 48, 14), Qt.AlignCenter, f'{v:g}')
        for i in range(int(round((y1 - y0) / ystep)) + 1):
            v = y0 + i * ystep
            py = to_px(x0, v).y()
            p.setPen(QPen(QColor('#9ca3af' if abs(v) < 1e-9 else '#e5e7eb'), 1))
            p.drawLine(QPointF(plot.left(), py), QPointF(plot.right(), py))
            p.setPen(QColor('#6b7280'))
            p.drawText(QRectF(0, py - 7, self.MARGIN_L - 6, 14), Qt.AlignRight | Qt.AlignVCenter,
                       f'{v:g}')
        p.setPen(QPen(QColor('#9ca3af'), 1))
        p.drawRect(plot)
        p.setPen(QColor('#374151'))
        p.drawText(QRectF(plot.left(), self.height() - 18, plot.width(), 16), Qt.AlignCenter,
                   X_LABELS[self.kind])
        p.save()
        p.translate(12, plot.center().y())
        p.rotate(-90)
        p.drawText(QRectF(-60, -8, 120, 16), Qt.AlignCenter, 'Fz [N]')
        p.restore()

        if not pts:
            p.setPen(QColor('#9ca3af'))
            p.drawText(plot, Qt.AlignCenter, '측정 데이터가 오면 여기에 그려집니다')
            return

        # 다른 모듈은 회색, 현재 모듈은 강조색으로 마지막에 그린다
        order = [m for m in self.traces if m != self.current]
        if self.current in self.traces:
            order.append(self.current)
        for mid in order:
            tr = self.traces[mid]
            if not tr:
                continue
            cur = mid == self.current
            p.setPen(QPen(QColor(ACCENT if cur else '#b6bcc6'), 2.2 if cur else 1.2))
            p.drawPolyline(QPolygonF([to_px(x, f) for x, f in tr]))
            end = to_px(*tr[-1])
            p.setPen(QColor(ACCENT if cur else '#8b93a0'))
            p.drawText(QRectF(end.x() + 4, end.y() - 8, 40, 16),
                       Qt.AlignLeft | Qt.AlignVCenter, f'M{mid}')
