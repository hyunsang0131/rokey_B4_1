# 화면(HMI): 모드 선택, 시작/정지, 6칸 히트맵(1차 측정 / 재측정), 교체 계획, 재고, 로그
#
#   ros2 run bed_inspect bed_hmi
#
# 화면은 "보여주기만" 한다. 로봇을 움직이는 것은 bed_process 이고,
# 화면은 /bed/cmd 로 명령을 보내고 /bed/status 로 받은 내용을 그대로 그린다.
# 그래서 화면을 껐다 켜도 공정에는 영향이 없고, 마지막 상태가 다시 표시된다.
#
# 색은 상태를 뜻한다: 초록 = 정상, 노랑 = 주의, 빨강 = 불량, 보라 = 빈칸, 주황 = 맞지 않는 모듈(자동 보수).
# 색만으로 구분하지 않도록 칸마다 기호와 글자(✓ 정상 / ! 주의 / ✕ 불량)를 같이 쓴다.

import json
import sys

from bed_inspect import bed_config as cfg

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from bed_inspect.bed_config import STEP_NAMES, STEP_REMEASURE

# ── 색 ────────────────────────────────────────────────────────────────
SURFACE = '#fcfcfb'
PANEL = '#ffffff'
INK = '#0b0b0b'
INK2 = '#5b5a55'
LINE = '#e1e0d9'
EMPTY = '#efeee9'
GRADE = {       # 등급: (배경, 글자색, 기호 + 이름)
    'ok': ('#0ca30c', '#ffffff', '✓ 정상'),
    'warn': ('#fab219', '#0b0b0b', '! 주의'),
    'fail': ('#d03b3b', '#ffffff', '✕ 불량'),
    'nodata': ('#8a8983', '#ffffff', '? 측정 실패'),
    'empty': ('#7a4fd0', '#ffffff', '○ 빈칸'),
    'mismatch': ('#e0701f', '#ffffff', '≠ 맞지 않음'),
}
CLS_NAMES = {'soft': '부드러움', 'hard': '단단함', 'tall': '높음'}
STATE = {       # 공정 상태: (이름, 배경, 글자색)
    'starting': ('시작 중', EMPTY, INK), 'idle': ('대기', EMPTY, INK),
    'running': ('실행 중', '#2a78d6', '#ffffff'), 'confirm': ('확인 대기', '#fab219', INK),
    'done': ('완료', '#0ca30c', '#ffffff'), 'warning': ('확인 필요', '#fab219', INK),
    'error': ('오류', '#d03b3b', '#ffffff'), 'stopped': ('정지', '#8a8983', '#ffffff'),
    None: ('연결 안 됨', EMPTY, INK2),
}

STYLE = f'''
QWidget {{ background: {SURFACE}; color: {INK}; font-size: 13px; }}
QGroupBox {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 8px; margin-top: 18px; padding: 10px 10px 8px 10px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; top: 2px; color: {INK2}; font-weight: bold; background: transparent; }}
QGroupBox QWidget {{ background: {PANEL}; }}
QPushButton {{ background: {PANEL}; border: 1px solid #c3c2b7; border-radius: 6px; padding: 7px 12px; }}
QPushButton:hover {{ background: #f3f2ee; }}
QPushButton:disabled {{ color: #b0afa8; border-color: {LINE}; }}
QPushButton#start {{ background: #2a78d6; color: #ffffff; border: none; font-weight: bold; padding: 10px; }}
QPushButton#start:disabled {{ background: #c9d9ee; }}
QPushButton#stop {{ background: #d03b3b; color: #ffffff; border: none; font-weight: bold; padding: 10px; }}
QPushButton#stop:disabled {{ background: #ecc9c9; }}
QPlainTextEdit, QListWidget {{ background: {PANEL}; border: 1px solid {LINE}; border-radius: 6px; }}
QComboBox, QDoubleSpinBox {{ background: {PANEL}; border: 1px solid #c3c2b7; border-radius: 4px; padding: 3px 6px; }}
QLabel#hint {{ color: {INK2}; font-size: 12px; }}
'''


RACK_PER_ROW = 3                # 재고 수정 창에서 보관대를 한 줄에 몇 자리씩 (트레이 1개 = 3자리)


def grid_pos(slot, cols):
    """칸 번호(1부터) → 화면 위치 (줄, 칸). bed_config.DISPLAY_GRID 를 따르고, 없으면 번호 순서대로."""
    for r, line in enumerate(cfg.DISPLAY_GRID):
        if slot in line:
            return r, line.index(slot)
    return (slot - 1) // cols, (slot - 1) % cols


def fmt(v, spec, dash='-'):
    return dash if v is None else format(v, spec)


def deg(a):
    """각도 표시: 0 은 부호 없이, 나머지는 +/- 를 붙인다."""
    return '0.0°' if abs(a) < 0.05 else f'{a:+.1f}°'


class Cell(QtWidgets.QFrame):
    """히트맵의 한 칸."""

    def __init__(self, slot):
        super().__init__()
        self.slot = slot
        self.setMinimumSize(132, 112)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)
        self.head = QtWidgets.QLabel()
        self.grade = QtWidgets.QLabel()
        self.line1 = QtWidgets.QLabel()
        self.line2 = QtWidgets.QLabel()
        for w in (self.head, self.grade, self.line1, self.line2):
            w.setAttribute(Qt.WA_TransparentForMouseEvents)
            lay.addWidget(w)
        self.line2.setWordWrap(True)
        lay.addStretch(1)
        self.show_block(None, None, False)

    def show_block(self, b, height, active):
        """b: 측정 결과(없으면 None), height: 재고에 기록된 높이, active: 지금 측정 중인 칸."""
        h = b['height'] if b else height
        self.head.setText(f'{self.slot}번' + (f' · {h:g}mm' if h is not None else ''))
        grade = b.get('grade') if b else None
        if b and grade is None:
            # 방금 잰 칸: 판정은 6칸을 다 잰 뒤에 나온다
            bg, fg, text = EMPTY, INK, '판정 대기' if b.get('k') is not None else '? 측정 실패'
        elif grade:
            bg, fg, text = GRADE[grade]
        else:
            bg, fg, text = EMPTY, INK2, '측정 중…' if active else '–'
        border = f'3px solid {INK}' if active else f'1px solid {LINE}'
        self.setStyleSheet(f'Cell {{ background: {bg}; border: {border}; border-radius: 8px; }}'
                           f'QLabel {{ background: transparent; color: {fg}; }}')
        self.head.setStyleSheet('font-size: 12px;')
        self.grade.setStyleSheet('font-size: 19px; font-weight: bold;')
        self.grade.setText(text)
        if not b:
            self.line1.setText('')
            self.line2.setText('')
            self.setToolTip('')
            return
        self.line1.setText(f'강성 {fmt(b.get("k"), ".2f")} N/mm')
        dev = b.get('dev')
        what = {'others': '주변 대비', 'profile': '목표 대비'}.get(b.get('ref_src'), '기준 대비')
        if b.get('want') and b.get('cls'):
            text = f'{CLS_NAMES[b["cls"]]} · 목표 {cfg.PROFILE_NAMES[b["want"]]}'
        elif b.get('tall'):
            text = '높은 큐브'
        elif b.get('too_soft'):
            text = '아주 무름 (끝까지 들어감)'
        elif b.get('grade') == 'empty':
            text = '모듈 없음'
        else:
            text = b.get('note') or ('' if dev is None else f'{what} {dev * 100:+.1f}%')
        self.line2.setText(text)
        src = {'calib': '저장한 기준값', 'median': '이번 측정의 중앙값', 'others': '다른 칸들의 중앙값',
               'profile': '원하는 종류의 대표 강성'}.get(b.get('ref_src'), '-')
        self.setToolTip(
            f'{self.slot}번 블록 ({fmt(b.get("height"), "g", "높이 기록 없음")}mm)\n'
            f'강성: {fmt(b.get("k"), ".3f")} N/mm\n'
            f'기준 강성: {fmt(b.get("ref"), ".3f")} N/mm ({src})\n'
            f'표면 높이: {fmt(b.get("z_surf"), ".2f")} mm\n'
            f'눌린 깊이: {fmt(b.get("disp"), ".1f")} mm\n'
            f'최대 힘: {fmt(b.get("f_max"), ".1f")} N\n'
            f'직선 맞춤 품질(R²): {fmt(b.get("r2"), ".3f")}'
            + (f'\n비고: {b["note"]}' if b.get('note') else ''))


class Heatmap(QtWidgets.QGroupBox):
    """측정 한 번의 결과: 6칸 + 기울기."""

    def __init__(self, title, rows, cols):
        super().__init__(title)
        lay = QtWidgets.QVBoxLayout(self)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        self.cells = []
        for i in range(rows * cols):              # cells[i] = i+1번 칸 (화면 위치는 grid_pos)
            c = Cell(i + 1)
            self.cells.append(c)
            grid.addWidget(c, *grid_pos(i + 1, cols))
        lay.addLayout(grid, 1)
        self.tilt = QtWidgets.QLabel('기울기: -')
        self.tilt.setObjectName('hint')
        lay.addWidget(self.tilt)

    def show_result(self, res, bed, active):
        blocks = {b['slot']: b for b in (res or {}).get('blocks', [])}
        for c in self.cells:
            h = bed[c.slot - 1] if bed and c.slot <= len(bed) else None
            c.show_block(blocks.get(c.slot), h, active == c.slot)
        t = (res or {}).get('tilt')
        if t:
            self.tilt.setText(f'기울기: {cfg.AXIS_NAMES["col"]} {deg(t["col_deg"])}, '
                              f'{cfg.AXIS_NAMES["row"]} {deg(t["row_deg"])}'
                              f'   (평면에서 벗어남 {t["rms"]:.1f}mm)')
        else:
            self.tilt.setText('기울기: -')


class InventoryDialog(QtWidgets.QDialog):
    """재고 수정: 실제 블록 위치를 사람이 확인해서 입력한다."""

    def __init__(self, parent, status):
        super().__init__(parent)
        self.setWindowTitle('재고 수정')
        heights = status['layout']['heights']
        inv, world = status['inventory'], status.get('world', {})
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(QtWidgets.QLabel('실제로 놓여 있는 블록 높이를 그대로 입력하세요.'))
        form = QtWidgets.QGridLayout()
        self.bed, self.rack = [], []
        cols = status['layout']['cols']
        form.addWidget(QtWidgets.QLabel('침대'), 0, 0)
        for i, h in enumerate(inv['bed']):
            cb = self._combo(heights, h, True)         # 빈자리 채우기 모드 때문에 침대 칸도 '비어 있음' 가능
            self.bed.append(cb)
            r, c = grid_pos(i + 1, cols)
            form.addWidget(QtWidgets.QLabel(f'{i + 1}번'), 1 + 2 * r, c)
            form.addWidget(cb, 2 + 2 * r, c)
        base = 3 + 2 * max(grid_pos(i + 1, cols)[0] for i in range(len(inv['bed'])))
        form.addWidget(QtWidgets.QLabel('보관대'), base, 0)
        for j, h in enumerate(inv['rack']):
            cb = self._combo(heights, h, True)
            self.rack.append(cb)
            form.addWidget(QtWidgets.QLabel(f'{j + 1}번'), base + 1 + 2 * (j // RACK_PER_ROW), j % RACK_PER_ROW)
            form.addWidget(cb, base + 2 + 2 * (j // RACK_PER_ROW), j % RACK_PER_ROW)
        lay.addLayout(form)
        self.cover = QtWidgets.QCheckBox('커버가 열려 있음')
        self.cover.setChecked(bool(world.get('cover_open')))
        lay.addWidget(self.cover)
        if not cfg.COVER_ENABLED:                      # 커버가 없으면 열림/닫힘도 없다
            self.cover.setChecked(False)
            self.cover.hide()
        box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        box.button(QtWidgets.QDialogButtonBox.Save).setText('저장')
        box.button(QtWidgets.QDialogButtonBox.Cancel).setText('취소')
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)
        lay.addWidget(box)

    @staticmethod
    def _combo(heights, value, allow_empty):
        cb = QtWidgets.QComboBox()
        if allow_empty:
            cb.addItem('비어 있음', None)
        for h in heights:
            cb.addItem(f'{h:g}mm', h)
        cb.setCurrentIndex(max(cb.findData(value), 0))
        return cb

    def command(self):
        return {'cmd': 'set_inventory', 'bed': [c.currentData() for c in self.bed],
                'rack': [c.currentData() for c in self.rack],
                'cover_open': self.cover.isChecked()}


class Toggle(QtWidgets.QWidget):
    """노션의 ▸ 토글: 머리줄을 누르면 아래 내용(body)이 펼쳐지고 접힌다."""

    def __init__(self, title, style=''):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.head = QtWidgets.QToolButton()
        self.head.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.head.setArrowType(Qt.RightArrow)
        self.head.setCheckable(True)
        self.head.setAutoRaise(True)
        self.head.setText(title)
        self.head.setStyleSheet('QToolButton { border: none; padding: 4px 2px; text-align: left; '
                                + ('color: #5b5a55; ' if style == 'header' else '') + '}')
        self.head.toggled.connect(self.set_open)
        self.head.setMinimumHeight(26)
        self.body = QtWidgets.QWidget()
        self.body.setVisible(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        lay.addWidget(self.head)
        lay.addWidget(self.body)

    def set_open(self, on):
        if self.head.isChecked() != on:
            self.head.setChecked(on)            # 다시 이 함수로 들어온다
            return
        self.head.setArrowType(Qt.DownArrow if on else Qt.RightArrow)
        self.body.setVisible(on)
        self.updateGeometry()                   # 펼친 만큼 자리를 다시 잡게


class ProfileRow(Toggle):
    """고객 맞춤의 칸 하나: '▸ 1번 칸 · 단단하게'. 펼치면 세 가지 중 하나를 고른다."""

    def __init__(self, i):
        super().__init__('')
        self.i = i
        self.value = 'hard'
        lay = QtWidgets.QHBoxLayout(self.body)
        lay.setContentsMargins(22, 0, 0, 4)
        self.opts = {}
        grp = QtWidgets.QButtonGroup(self)
        for v in ('soft', 'hard', 'tall'):
            rb = QtWidgets.QRadioButton(cfg.PROFILE_NAMES[v])
            rb.toggled.connect(lambda on, v=v: on and self.set_value(v))
            grp.addButton(rb)
            lay.addWidget(rb)
            self.opts[v] = rb
        lay.addStretch(1)
        self.body.setMinimumHeight(28)
        self.set_value('hard')

    def set_value(self, value):
        self.value = value
        if not self.opts[value].isChecked():
            self.opts[value].setChecked(True)
        color = {'hard': INK, 'soft': '#2a78d6', 'tall': '#7a4fd0'}[value]
        self.head.setText(f'{self.i + 1}번 칸 · {cfg.PROFILE_NAMES[value]}')
        self.head.setStyleSheet(f'QToolButton {{ border: none; padding: 4px 2px; color: {color}; }}')


class BedWindow(QtWidgets.QWidget):
    def __init__(self, send, rows=cfg.BED_ROWS, cols=cfg.BED_COLS):
        """send: 명령 dict 를 받아 /bed/cmd 로 보내는 함수."""
        super().__init__()
        self.send = send
        self.status = None
        self.setWindowTitle('실리콘 침대 검사')
        self.setStyleSheet(STYLE)
        self.resize(1360, 820)
        root = QtWidgets.QVBoxLayout(self)

        # 맨 위: 상태와 안내 문장
        top = QtWidgets.QHBoxLayout()
        self.badge = QtWidgets.QLabel()
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setFixedSize(110, 36)
        self.message = QtWidgets.QLabel()
        self.message.setWordWrap(True)
        self.message.setStyleSheet('font-size: 15px;')
        top.addWidget(self.badge)
        top.addWidget(self.message, 1)
        root.addLayout(top)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)
        body.addWidget(self._controls(), 0)

        mid = QtWidgets.QVBoxLayout()
        maps = QtWidgets.QHBoxLayout()
        self.map1 = Heatmap('1차 측정', rows, cols)
        self.map2 = Heatmap('재측정 (교체 후)', rows, cols)
        maps.addWidget(self.map1)
        maps.addWidget(self.map2)
        mid.addLayout(maps, 1)
        self.legend = QtWidgets.QLabel()
        self.legend.setTextFormat(Qt.RichText)
        mid.addWidget(self.legend)
        body.addLayout(mid, 1)
        body.addWidget(self._side(), 0)
        self.apply_status(None)

    # ── 화면 구성 ─────────────────────────────────────────────────────
    def _controls(self):
        w = QtWidgets.QWidget()
        w.setFixedWidth(290)
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)

        g = QtWidgets.QGroupBox('모드')
        gl = QtWidgets.QVBoxLayout(g)
        self.rb_repair = QtWidgets.QRadioButton('자동 보수 (주변 칸에 맞춤)')
        self.rb_user = QtWidgets.QRadioButton('고객 맞춤 (칸마다 선택)')
        self.rb_measure = QtWidgets.QRadioButton('측정만')
        self.rb_replace = QtWidgets.QRadioButton('불량 블록 교체')
        self.rb_repair.setChecked(True)
        grp = QtWidgets.QButtonGroup(g)                 # 접힌 곳에 있어도 네 개 중 하나만 고른다
        for rb in (self.rb_repair, self.rb_user, self.rb_measure, self.rb_replace):
            grp.addButton(rb)
            rb.toggled.connect(self._refresh_buttons)
        gl.addWidget(self.rb_repair)
        gl.addWidget(self.rb_user)
        gl.addWidget(self._repair_box())
        # 점검용 모드: 노션 토글처럼 접어 둔다
        self.more = Toggle('점검용 모드', 'header')
        more = QtWidgets.QVBoxLayout(self.more.body)
        more.setContentsMargins(18, 0, 0, 0)
        for rb in (self.rb_measure, self.rb_replace):
            more.addWidget(rb)
        gl.addWidget(self.more)
        self.confirm = QtWidgets.QCheckBox('교체 전에 계획을 확인')
        gl.addWidget(self.confirm)
        lay.addWidget(g)

        self.b_start = QtWidgets.QPushButton('시작')
        self.b_start.setObjectName('start')
        self.b_stop = QtWidgets.QPushButton('정지')
        self.b_stop.setObjectName('stop')
        self.b_go = QtWidgets.QPushButton('진행')
        self.b_cancel = QtWidgets.QPushButton('취소')
        self.b_home = QtWidgets.QPushButton('홈으로')
        self.b_calib = QtWidgets.QPushButton('기준값 저장')
        self.b_inv = QtWidgets.QPushButton('재고 수정…')
        self.b_start.clicked.connect(self._start)
        self.b_stop.clicked.connect(lambda: self.send({'cmd': 'stop'}))
        self.b_go.clicked.connect(lambda: self.send({'cmd': 'confirm'}))
        self.b_cancel.clicked.connect(lambda: self.send({'cmd': 'cancel'}))
        self.b_home.clicked.connect(lambda: self.send({'cmd': 'home'}))
        self.b_calib.clicked.connect(self._calibrate)
        self.b_inv.clicked.connect(self._edit_inventory)
        lay.addWidget(self.b_start)
        lay.addWidget(self.b_stop)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.b_go)
        row.addWidget(self.b_cancel)
        lay.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.b_home)
        row.addWidget(self.b_calib)
        lay.addLayout(row)
        lay.addWidget(self.b_inv)

        g = QtWidgets.QGroupBox('진행 단계')
        gl = QtWidgets.QVBoxLayout(g)
        gl.setSpacing(1)
        self.steps = []
        for _ in range(len(STEP_NAMES) - 1):
            lb = QtWidgets.QLabel()
            self.steps.append(lb)
            gl.addWidget(lb)
        lay.addWidget(g)
        lay.addStretch(1)
        return w

    def _repair_box(self):
        """고객 맞춤: 칸마다 ▸ 토글을 펼쳐 부드럽게 / 단단하게 / 높게 중 하나를 고른다."""
        self.repair_box = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(self.repair_box)
        lay.setContentsMargins(18, 0, 0, 4)
        lay.setSpacing(0)
        self.profile = []
        for i in range(len(cfg.P_BED)):
            row = ProfileRow(i)
            self.profile.append(row)
            lay.addWidget(row)
        return self.repair_box

    def set_profile(self, i, value):
        """i번 칸(0부터)의 원하는 종류를 바꾼다: 'hard' / 'soft' / 'tall'."""
        self.profile[i].set_value(value)

    def profile_values(self):
        return [r.value for r in self.profile]

    def _side(self):
        w = QtWidgets.QWidget()
        w.setFixedWidth(330)
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        g = QtWidgets.QGroupBox('교체 계획')
        gl = QtWidgets.QVBoxLayout(g)
        self.plan_head = QtWidgets.QLabel('-')
        self.plan_head.setWordWrap(True)
        self.plan_list = QtWidgets.QLabel('')
        self.plan_list.setWordWrap(True)
        gl.addWidget(self.plan_head)
        gl.addWidget(self.plan_list)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox('재고')
        gl = QtWidgets.QVBoxLayout(g)
        self.inv = QtWidgets.QLabel('-')
        self.inv.setWordWrap(True)
        gl.addWidget(self.inv)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox('로그')
        gl = QtWidgets.QVBoxLayout(g)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        gl.addWidget(self.log)
        lay.addWidget(g, 1)
        return w

    # ── 버튼 ──────────────────────────────────────────────────────────
    def _mode(self):
        if self.rb_measure.isChecked():
            return 'measure'
        if self.rb_repair.isChecked() or self.rb_user.isChecked():
            return 'repair'
        return 'replace'

    def _start(self):
        cmd = {'cmd': 'start', 'mode': self._mode(), 'confirm': self.confirm.isChecked()}
        if cmd['mode'] == 'repair':
            cmd['target'] = 'user' if self.rb_user.isChecked() else 'auto'
            if cmd['target'] == 'user':
                cmd['profile'] = self.profile_values()
        self.send(cmd)

    def _calibrate(self):
        q = QtWidgets.QMessageBox.question(
            self, '기준값 저장', '지금 침대에 있는 블록 6개가 전부 정상일 때만 하세요.\n'
            '6개를 측정해서 높이별 기준 강성으로 저장합니다. 진행할까요?')
        if q == QtWidgets.QMessageBox.Yes:
            self.send({'cmd': 'calibrate'})

    def _edit_inventory(self):
        if not self.status:
            return
        dlg = InventoryDialog(self, self.status)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            self.send(dlg.command())

    def _refresh_buttons(self):
        st = self.status['state'] if self.status else None
        idle = st in ('idle', 'done', 'warning', 'error', 'stopped')
        self.repair_box.setVisible(self.rb_user.isChecked())
        if self.rb_measure.isChecked() or self.rb_replace.isChecked():
            self.more.set_open(True)                    # 점검용 모드를 골랐으면 펼쳐 둔다
        self.confirm.setEnabled(not self.rb_measure.isChecked())
        self.b_start.setEnabled(idle)
        self.b_stop.setEnabled(st in ('running', 'confirm'))
        self.b_go.setVisible(st == 'confirm')
        self.b_cancel.setVisible(st == 'confirm')
        for b in (self.b_home, self.b_calib, self.b_inv):
            b.setEnabled(idle)

    # ── 상태 표시 ─────────────────────────────────────────────────────
    def add_log(self, text):
        self.log.appendPlainText(text)
        bar = self.log.verticalScrollBar()
        bar.setValue(bar.maximum())             # 항상 최신 줄이 보이게

    def apply_status(self, s):
        """bed_process 가 보낸 상태 전체를 화면에 그린다."""
        self.status = s
        name, bg, fg = STATE.get(s['state'] if s else None, STATE[None])
        self.badge.setText(name)
        self.badge.setStyleSheet(f'background: {bg}; color: {fg}; border-radius: 8px; font-weight: bold; font-size: 15px;')
        if not s:
            self.message.setText('공정 노드(bed_process)를 기다리는 중입니다.')
            self.map1.show_result(None, None, None)
            self.map2.show_result(None, None, None)
            self._show_steps(0, None, STEP_NAMES)
            self._refresh_buttons()
            self.legend.setText('')
            return
        detail = f'{s["step"]}단계 {s["step_name"]}' + (f' · {s["detail"]}' if s.get('detail') else '')
        self.message.setText(s.get('message') or (detail if s['state'] == 'running' else ''))

        bed = s['inventory']['bed']
        second_active = s['step'] >= STEP_REMEASURE
        self.map1.show_result(s.get('first'), bed if not s.get('second') else None,
                              None if second_active else s.get('active_slot'))
        self.map2.show_result(s.get('second'), bed if second_active else None,
                              s.get('active_slot') if second_active else None)
        self._show_steps(s['step'], s['state'], s.get('step_names') or STEP_NAMES)
        self._show_plan(s.get('plan'))
        rack = ', '.join(f'{j + 1}번 ' + ('비어 있음' if h is None else f'{h:g}mm')
                         for j, h in enumerate(s['inventory']['rack']))
        calib = ', '.join(f'{float(h):g}mm = {k:.2f}' for h, k in sorted((s.get('calib') or {}).items(),
                                                                       key=lambda x: float(x[0])))
        w = s.get('world', {})
        self.inv.setText(
            '침대: ' + ' / '.join(f'{h:g}' if h is not None else '빈칸' for h in bed) + ' mm\n'
            f'보관대: {rack}\n'
            f'커버: {("열림" if w.get("cover_open") else "닫힘") if cfg.COVER_ENABLED else "없음"}\n'
            f'기준 강성 [N/mm]: {calib or "저장 안 됨 (이번 측정의 중앙값과 비교)"}')
        lay = s['layout']
        sw = '<span style="background:{0}; color:{0};">■■</span> {1}'
        self.legend.setText('&nbsp;&nbsp;&nbsp;'.join([
            sw.format(GRADE['ok'][0], f'✓ 정상 (기준 강성과 차이 {lay["warn"] * 100:.0f}% 미만)'),
            sw.format(GRADE['warn'][0], f'! 주의 ({lay["warn"] * 100:.0f}~{lay["fail"] * 100:.0f}%)'),
            sw.format(GRADE['fail'][0], f'✕ 불량 ({lay["fail"] * 100:.0f}% 이상, 교체 대상)'),
            sw.format(GRADE['nodata'][0], '? 측정 실패'),
            sw.format(GRADE['empty'][0], '○ 빈칸'),
            sw.format(GRADE['mismatch'][0], '≠ 맞지 않음 (자동 보수)')]))
        self._refresh_buttons()

    def _show_steps(self, cur, state, names):
        for n, lb in enumerate(self.steps, 1):
            running = state in ('running', 'confirm')
            if n < cur or (n == cur and not running and state in ('done', 'warning')):
                mark, style = '✓', f'color: {INK2};'
            elif n == cur:
                mark, style = '▶', 'font-weight: bold;'
            else:
                mark, style = '○', f'color: {INK2};'
            lb.setText(f'{mark}  {n}. {names[n] if n < len(names) else ""}')
            lb.setStyleSheet(style)

    def _show_repair_plan(self, p):
        head, lines = _repair_plan_text(p)
        done = p.get('done', 0)
        moves = p.get('moves') or []
        if moves:
            lines += ['이동:'] + [('✓ ' if i < done else '○ ') + t for i, t in enumerate(moves)]
        self.plan_head.setText(head)
        self.plan_list.setText('\n'.join(lines))

    def _show_plan(self, p):
        if not p:
            self.plan_head.setText('-')
            self.plan_list.setText('')
            return
        if p['type'] == 'repair':
            self._show_repair_plan(p)
            return
        head = f'불량 {p["faulty"]}번을 같은 높이 예비 블록으로 교체'
        if p.get('skipped'):
            head += f'\n예비 블록이 없어 못 바꾸는 칸: {p["skipped"]}번'
        self.plan_head.setText(head)
        done = p.get('done', 0)
        moves = p.get('moves') or []
        self.plan_list.setText('\n'.join(('✓ ' if i < done else '○ ') + t for i, t in enumerate(moves))
                               or '옮길 블록 없음')


def _repair_plan_text(p):
    """자동 보수 계획을 (머리말, 목록) 글자로."""
    if p.get('target_mode') == 'user':
        head = '고객 맞춤 (' + ' / '.join(f'{i + 1}번 {cfg.PROFILE_NAMES[v]}'
                                             for i, v in enumerate(p.get('profile') or [])) + ')'
    else:
        head = '자동 보수 (주변 칸 강성에 맞춤)'
    head += f'\n빈칸 {p.get("empties") or "없음"} / 맞지 않는 칸 {p.get("mismatch") or "없음"}'
    lines = []
    if p.get('thr'):
        lines.append(f'종류 경계 {p["thr"]:.2f} N/mm')
    if p.get('rack'):
        lines.append('보관대 큐브 측정 (침대 기준 환산):')
        chosen = {c['rack'] for c in p.get('choice', [])}
        for r in p['rack']:
            mark = '★ ' if r['slot'] in chosen else '   '
            if r.get('empty'):
                val = '비어 있음'
            elif r.get('tall'):
                val = '높음 (살짝 닿기' + (f', +{r["dz"]:.1f} mm)' if r.get('dz') is not None else ')')
            elif r.get('k_bed') is None:
                val = (f'측정 실패 ({r["note"]})' if r.get('note')
                       else '닿음, 누르기 대기' if r.get('z0') is not None else '대기')
            else:
                val = f'{r["k_bed"]:.2f} N/mm'
                if r.get('cls'):
                    val += ' ' + CLS_NAMES[r['cls']]
                elif r.get('tall'):
                    val += ' 높음'
            lines.append(f'{mark}{r["slot"]}번: {val}')
    if p.get('choice'):
        lines.append('선택:')
        lines += [f'  보관대 {c["rack"]}번 → {c["slot"]}번 ' + ('(교체)' if c.get('why') == 'mismatch' else '(빈칸)')
                  for c in p['choice']]
    return head, lines


def make_app(args=None):
    """ROS 노드와 화면을 만들어 연결한다. (app, win, node, timer) 를 돌려준다."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = rclpy.create_node('bed_hmi')
    pub = node.create_publisher(String, '/bed/cmd', 10)
    app = QtWidgets.QApplication(sys.argv)
    win = BedWindow(lambda cmd: pub.publish(String(data=json.dumps(cmd, ensure_ascii=False))))

    # 늦게 켜도 마지막 상태를 받도록 bed_process 와 같은 QoS 를 쓴다
    latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/bed/status', lambda m: win.apply_status(json.loads(m.data)), latch)
    node.create_subscription(String, '/bed/log', lambda m: win.add_log(m.data), 50)

    # ROS 통신을 Qt 이벤트 루프 안에서 20ms 마다 처리
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(20)
    return app, win, node, timer


def main(args=None):
    import rclpy
    app, win, node, timer = make_app(args)
    win.show()
    code = app.exec_()
    timer.stop()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
