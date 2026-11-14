# PocketPCR picture guide (unofficial) - ReportLab
# Usage: pip install reportlab && python3 build_guide.py out.pdf
# Needs the DejaVu Sans fonts (set FONT_DIR if they are elsewhere).
import sys
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
import os
FONT_DIR = os.environ.get('FONT_DIR', '/usr/share/fonts/truetype/dejavu')

pdfmetrics.registerFont(TTFont('F', FONT_DIR + '/DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('FB', FONT_DIR + '/DejaVuSans-Bold.ttf'))

W, H = A4
OUT = sys.argv[1]
c = canvas.Canvas(OUT, pagesize=A4)
c.setTitle('PocketPCR - picture assembly guide (unofficial)')
c.setAuthor('Redrawn from the official PocketPCR instructions (CC BY 4.0) and the GaudiLabs repo')

INK = colors.HexColor('#222222')
GREY = colors.HexColor('#9a9a9a')
LIGHT = colors.HexColor('#eeeeee')
PCB = colors.HexColor('#2b2b2b')
ALU = colors.HexColor('#c9ccd1')
BLUE = colors.HexColor('#1f6feb')
RED = colors.HexColor('#d1242f')
YEL = colors.HexColor('#f2c200')
GREEN = colors.HexColor('#2da44e')
ORANGE = colors.HexColor('#e36209')

page_no = [0]


def txt(x, y, s, size=10, font='F', col=INK, anchor='l'):
    c.setFillColor(col)
    c.setFont(font, size)
    if anchor == 'l':
        c.drawString(x, y, s)
    elif anchor == 'c':
        c.drawCentredString(x, y, s)
    else:
        c.drawRightString(x, y, s)


def para(x, y, lines, size=9.5, lead=13, font='F', col=INK):
    for i, l in enumerate(lines):
        txt(x, y - i * lead, l, size, font, col)
    return y - len(lines) * lead


def warn(x, y, s=8, label=None):
    """yellow warning triangle with optional note id"""
    p = c.beginPath()
    p.moveTo(x, y); p.lineTo(x + s * 2, y); p.lineTo(x + s, y + s * 1.75); p.close()
    c.setFillColor(YEL); c.setStrokeColor(INK); c.setLineWidth(0.8)
    c.drawPath(p, fill=1, stroke=1)
    txt(x + s, y + 2.5, '!', s * 1.1, 'FB', INK, 'c')
    if label:
        txt(x + s * 2 + 3, y + 2, label, 7.5, 'FB')


def arrow(x1, y1, x2, y2, col=BLUE, w=2.2, head=7):
    import math
    c.setStrokeColor(col); c.setFillColor(col); c.setLineWidth(w)
    c.line(x1, y1, x2, y2)
    a = math.atan2(y2 - y1, x2 - x1)
    p = c.beginPath()
    p.moveTo(x2, y2)
    p.lineTo(x2 - head * math.cos(a - 0.4), y2 - head * math.sin(a - 0.4))
    p.lineTo(x2 - head * math.cos(a + 0.4), y2 - head * math.sin(a + 0.4))
    p.close()
    c.drawPath(p, fill=1, stroke=0)


def box(x, y, w, h, title=None, r=6):
    c.setStrokeColor(INK); c.setLineWidth(1); c.setFillColor(colors.white)
    c.roundRect(x, y, w, h, r, fill=1, stroke=1)
    if title:
        txt(x + 10, y + h - 18, title, 11, 'FB')


def step_badge(x, y, n):
    c.setFillColor(INK)
    c.circle(x, y, 16, fill=1, stroke=0)
    txt(x, y - 6, str(n), 17, 'FB', colors.white, 'c')


def footer():
    page_no[0] += 1
    txt(18 * mm, 10 * mm, 'PocketPCR - unofficial picture guide - v1, Nov 2026', 7, col=GREY)
    txt(W - 18 * mm, 10 * mm, str(page_no[0]), 8, col=GREY, anchor='r')


def header(title, sub=None):
    txt(18 * mm, H - 22 * mm, title, 20, 'FB')
    if sub:
        txt(18 * mm, H - 29 * mm, sub, 9.5, col=colors.HexColor('#555555'))


def newpage():
    footer()
    c.showPage()


def cross(x, y, s=8):
    c.setStrokeColor(RED); c.setLineWidth(2.5)
    c.line(x - s, y - s, x + s, y + s); c.line(x - s, y + s, x + s, y - s)


def tick(x, y, s=8):
    c.setStrokeColor(GREEN); c.setLineWidth(2.8)
    c.line(x - s, y, x - s / 3, y - s * 0.7); c.line(x - s / 3, y - s * 0.7, x + s, y + s * 0.8)

# ---------- drawing primitives for the device (top view, real geometry) ----------
# Board is 105 x 33 mm. Positions (mm from the board's left/top edge) are taken
# from the KiCad layout in the repo:
#   USB-C J3 x=99.7  | encoder SW1 x=81.9 y=24.0 | OLED centre x=58.7 y=16.0
#   fan header J1 x=39.2 y=27.7 | heater connector J4 x=1.5


def board_top(x0, y0, s, fan_hdr=True, enc=True, show_block=True, labels=False, knob=False):
    """x0,y0 = top-left corner of board in page coords, s = points per mm"""
    def P(mx, my):
        return x0 + mx * s, y0 - my * s
    c.setFillColor(PCB); c.setStrokeColor(INK); c.setLineWidth(1)
    c.roundRect(x0, y0 - 33 * s, 105 * s, 33 * s, 2 * s, fill=1, stroke=1)
    # OLED
    ox, oy = P(42, 4)
    c.setFillColor(colors.HexColor('#0b1a2a')); c.setStrokeColor(GREY)
    c.rect(ox, oy - 22 * s, 34 * s, 22 * s, fill=1, stroke=1)
    c.setFillColor(colors.HexColor('#7fd3ff'))
    c.setFont('F', max(4, 2.6 * s)); c.drawString(ox + 4 * s, oy - 9 * s, 'Run PCR')
    c.drawString(ox + 7 * s, oy - 17 * s, 'Setup')
    # heat block (19.5 mm round)
    if show_block:
        bx, by = P(14, 16.5)
        c.setFillColor(ALU); c.setStrokeColor(INK)
        c.circle(bx, by, 9.75 * s, fill=1, stroke=1)
        c.setFillColor(colors.HexColor('#8d939b'))
        for ang in range(5):
            import math
            a = math.radians(90 + ang * 72)
            c.circle(bx + 5.6 * s * math.cos(a), by + 5.6 * s * math.sin(a), 2.0 * s, fill=1, stroke=0)
        c.setFillColor(colors.HexColor('#555555'))
        c.circle(bx, by, 1.3 * s, fill=1, stroke=0)
    # mounting holes: fan (24 mm square around the block) + 2 at the USB end (approx.)
    c.setFillColor(colors.white)
    for hx, hy in fan_holes() + rear_holes():
        hx2, hy2 = P(hx, hy)
        c.circle(hx2, hy2, 1.6 * s, fill=1, stroke=0)
    # USB-C
    ux, uy = P(101, 12.5)
    c.setFillColor(colors.HexColor('#b0b0b0'))
    c.roundRect(ux, uy - 9 * s, 4.5 * s, 9 * s, 1 * s, fill=1, stroke=0)
    # encoder footprint / encoder
    ex, ey = P(81.9, 24.0)
    if enc:
        c.setFillColor(colors.HexColor('#777777'))
        c.rect(ex - 6 * s, ey - 6 * s, 12 * s, 12 * s, fill=1, stroke=0)
        c.setFillColor(colors.HexColor('#cfcfcf'))
        c.circle(ex, ey, 3 * s, fill=1, stroke=0)
    else:
        c.setFillColor(colors.HexColor('#d4af37'))
        for dx in (-2.5, 0, 2.5):
            c.circle(ex + dx * s, ey - 7.5 * s, 0.6 * s, fill=1, stroke=0)
        for dx in (-2.5, 2.5):
            c.circle(ex + dx * s, ey + 7 * s, 0.6 * s, fill=1, stroke=0)
        for dx in (-5.7, 5.7):
            c.rect(ex + dx * s - 0.6 * s, ey - 1.5 * s, 1.2 * s, 3 * s, fill=1, stroke=0)
    if knob:
        c.setFillColor(ALU); c.setStrokeColor(INK)
        c.circle(ex, ey, 6.5 * s, fill=1, stroke=1)
        c.setStrokeColor(GREY); c.circle(ex, ey, 4.5 * s, fill=0, stroke=1)
    # fan header
    if fan_hdr:
        fx, fy = P(39.2, 27.7)
        c.setFillColor(colors.HexColor('#d4af37'))
        c.circle(fx - 1.27 * s, fy, 0.7 * s, fill=1, stroke=0)
        c.circle(fx + 1.27 * s, fy, 0.7 * s, fill=1, stroke=0)
    if labels:
        lab = [('heat block', 14, -1), ('screen', 59, -1), ('knob', 82, 36), ('USB-C', 101, 36)]
        for t, mx, my in lab:
            px, py = P(mx, my)
            txt(px, py + (8 if my < 0 else -10), t, 8, col=INK, anchor='c')
    return P


def fan_holes():
    # 30 mm fan, 24 mm hole spacing, centred under the block
    cx, cy = 14 + 2, 16.5
    return [(cx - 12, cy - 12), (cx + 12, cy - 12), (cx - 12, cy + 12), (cx + 12, cy + 12)]


def rear_holes():
    return [(93, 4.5), (93, 28.5)]


def screw_side(x, y, s, length_mm, head='pan', col=colors.HexColor('#333333'), label=None, down=True):
    """screw drawn pointing down, head top at (x,y), s = pt/mm"""
    c.setFillColor(col); c.setStrokeColor(INK); c.setLineWidth(0.6)
    if head == 'pan':
        c.roundRect(x - 2.8 * s, y - 2.0 * s, 5.6 * s, 2.0 * s, 0.9 * s, fill=1, stroke=1)
        top = y - 2.0 * s
    elif head == 'thumb':
        c.rect(x - 3.5 * s, y - 4 * s, 7 * s, 4 * s, fill=1, stroke=1)
        c.setStrokeColor(GREY)
        for i in range(1, 7):
            c.line(x - 3.5 * s + i * s, y, x - 3.5 * s + i * s, y - 4 * s)
        top = y - 4 * s
    elif head == 'csk':
        p = c.beginPath(); p.moveTo(x - 2.8 * s, y); p.lineTo(x + 2.8 * s, y); p.lineTo(x + 1.5 * s, y - 1.6 * s); p.lineTo(x - 1.5 * s, y - 1.6 * s); p.close()
        c.drawPath(p, fill=1, stroke=1); top = y - 1.6 * s
    c.setFillColor(col); c.setStrokeColor(INK)
    c.rect(x - 1.5 * s, top - length_mm * s, 3 * s, length_mm * s, fill=1, stroke=1)
    c.setStrokeColor(GREY); c.setLineWidth(0.4)
    yy = top - 0.5 * s
    while yy > top - length_mm * s + 0.3 * s:
        c.line(x - 1.5 * s, yy, x + 1.5 * s, yy - 0.35 * s)
        yy -= 0.5 * s
    if label:
        txt(x, top - length_mm * s - 12, label, 8, anchor='c')


def nut_side(x, y, s, label=None):
    c.setFillColor(colors.HexColor('#999999')); c.setStrokeColor(INK); c.setLineWidth(0.6)
    c.rect(x - 2.75 * s, y - 2.4 * s, 5.5 * s, 2.4 * s, fill=1, stroke=1)
    c.line(x - 1.4 * s, y, x - 1.4 * s, y - 2.4 * s); c.line(x + 1.4 * s, y, x + 1.4 * s, y - 2.4 * s)
    if label:
        txt(x, y - 2.4 * s - 12, label, 8, anchor='c')


def nut_top(x, y, s):
    import math
    p = c.beginPath()
    for i in range(6):
        a = math.radians(60 * i)
        px, py = x + 3.17 * s * math.cos(a), y + 3.17 * s * math.sin(a)
        (p.moveTo if i == 0 else p.lineTo)(px, py)
    p.close()
    c.setFillColor(colors.HexColor('#aaaaaa')); c.setStrokeColor(INK); c.setLineWidth(0.6)
    c.drawPath(p, fill=1, stroke=1)
    c.setFillColor(colors.white); c.circle(x, y, 1.5 * s, fill=1, stroke=1)


def spacer_side(x, y, s, L, label=None):
    c.setFillColor(colors.HexColor('#c8a85a')); c.setStrokeColor(INK); c.setLineWidth(0.6)
    c.rect(x - 2.75 * s, y - L * s, 5.5 * s, L * s, fill=1, stroke=1)
    c.line(x - 1.4 * s, y, x - 1.4 * s, y - L * s); c.line(x + 1.4 * s, y, x + 1.4 * s, y - L * s)
    if label:
        txt(x, y - L * s - 12, label, 8, anchor='c')


def foot_side(x, y, s):
    c.setFillColor(colors.HexColor('#111111')); c.setStrokeColor(INK)
    c.roundRect(x - 4 * s, y - 3 * s, 8 * s, 3 * s, 1 * s, fill=1, stroke=1)
    c.rect(x - 1.3 * s, y, 2.6 * s, 2.5 * s, fill=1, stroke=1)


def fan_side(x, y, s):
    c.setFillColor(colors.HexColor('#3a3a3a')); c.setStrokeColor(INK)
    c.rect(x, y - 10 * s, 30 * s, 10 * s, fill=1, stroke=1)
    c.setStrokeColor(GREY)
    for i in range(1, 6):
        c.line(x + i * 5 * s, y - 1 * s, x + i * 5 * s, y - 9 * s)


def fan_top(x, y, s, cable=True):
    """x,y = top-left"""
    c.setFillColor(colors.HexColor('#3a3a3a')); c.setStrokeColor(INK); c.setLineWidth(1)
    c.roundRect(x, y - 30 * s, 30 * s, 30 * s, 2 * s, fill=1, stroke=1)
    c.setFillColor(colors.HexColor('#555555')); c.circle(x + 15 * s, y - 15 * s, 13 * s, fill=1, stroke=0)
    c.setFillColor(colors.HexColor('#2a2a2a')); c.circle(x + 15 * s, y - 15 * s, 6 * s, fill=1, stroke=0)
    c.setStrokeColor(GREY)
    import math
    for i in range(7):
        a = math.radians(i * 360 / 7)
        c.line(x + 15 * s + 6 * s * math.cos(a), y - 15 * s + 6 * s * math.sin(a),
               x + 15 * s + 12.5 * s * math.cos(a + 0.5), y - 15 * s + 12.5 * s * math.sin(a + 0.5))
    c.setFillColor(colors.white)
    for hx, hy in [(3, 3), (27, 3), (3, 27), (27, 27)]:
        c.circle(x + hx * s, y - hy * s, 1.6 * s, fill=1, stroke=0)
    if cable:
        c.setStrokeColor(RED); c.setLineWidth(1.4)
        c.line(x + 30 * s, y - 26 * s, x + 38 * s, y - 26 * s)
        c.setStrokeColor(colors.black)
        c.line(x + 30 * s, y - 27.3 * s, x + 38 * s, y - 27.3 * s)
        c.setFillColor(colors.white); c.setStrokeColor(INK); c.setLineWidth(0.7)
        c.rect(x + 38 * s, y - 29 * s, 5 * s, 5 * s, fill=1, stroke=1)


# =============================== PAGE 1: cover ===============================
txt(18 * mm, H - 35 * mm, 'PocketPCR', 34, 'FB')
txt(18 * mm, H - 45 * mm, 'Picture assembly + use guide', 16)
txt(18 * mm, H - 53 * mm, 'Unofficial redraw of the GaudiLabs instructions, with India sourcing', 10, col=colors.HexColor('#555555'))

P = board_top(30 * mm, H - 90 * mm, 4.4, enc=True, knob=True, labels=True)
# lid on block
bx, by = P(14, 16.5)
c.setFillColor(colors.white); c.setStrokeColor(INK); c.setLineWidth(1)
c.circle(bx, by, 10.5 * 4.4, fill=0, stroke=1)
txt(30 * mm, H - 90 * mm - 33 * 4.4 - 20, 'Finished PocketPCR seen from above (33 x 105 x 50 mm, 50 g)', 9, col=colors.HexColor('#555555'))

y = H - 163 * mm
box(18 * mm, y - 62 * mm, W - 36 * mm, 70 * mm, 'At a glance')
yy = y - 6 * mm
rows = [('Time', 'about 30 minutes (plus printing the optional case)'),
        ('Skill', 'easy, but 2 small soldering jobs (7 joints in total)'),
        ('Power', '5 V USB-C, 2 A or more (phone charger or power bank)'),
        ('Holds', '5 x 0.2 ml PCR tubes, ambient to 99 °C'),
        ('Already done', 'electronics, screen and heat block come pre-assembled'),
        ('Not included', 'USB adapter and cable, 3D-printed case, soldering iron')]
for k, v in rows:
    txt(26 * mm, yy, k, 10, 'FB'); txt(62 * mm, yy, v, 10)
    yy -= 8.5 * mm

y2 = y - 70 * mm
box(18 * mm, y2 - 42 * mm, W - 36 * mm, 42 * mm, 'What is in this guide')
para(26 * mm, y2 - 12 * mm, ['p.2  Before you start: safety, tools, symbols', 'p.3  Parts in the box', 'p.4  Screws and spacers at actual size', 'p.5-7  Assembly steps 1-10'], 9.5, 13)
para(110 * mm, y2 - 12 * mm, ['p.8  Load samples and run a program', 'p.9  Screens and knob map', 'p.10  Problems and fixes', 'p.11  Buying it in India (₹)', 'p.12  Notes for the ⚠ flags'], 9.5, 13)
newpage()

# =============================== PAGE 2: before you start ===============================
header('Before you start')
# safety
y = H - 40 * mm
box(18 * mm, y - 55 * mm, 82 * mm, 55 * mm, 'Safety')
# hot icon
c.setFillColor(ORANGE); c.circle(32 * mm, y - 27 * mm, 8 * mm, fill=1, stroke=0)
txt(32 * mm, y - 29.5 * mm, '100°', 11, 'FB', colors.white, 'c')
para(44 * mm, y - 21 * mm, ['Heat block gets to', '100 °C. Don\'t touch', 'it during a run.'], 9, 12)
para(26 * mm, y - 44 * mm, ['Soldering iron tip is ~350 °C.', 'Work on a heat-proof mat, ventilate.'], 8.5, 11)

box(106 * mm, y - 55 * mm, W - 124 * mm, 55 * mm, 'Unplug while building')
# plug drawing
px, py = 125 * mm, y - 30 * mm
c.setFillColor(colors.HexColor('#444444')); c.roundRect(px, py, 16 * mm, 9 * mm, 2 * mm, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#bbbbbb')); c.rect(px + 16 * mm, py + 3 * mm, 5 * mm, 3 * mm, fill=1, stroke=0)
cross(px + 30 * mm, py + 4.5 * mm, 9)
para(112 * mm, y - 44 * mm, ['USB cable OUT until step 10.'], 9, 12)

# tools
y = H - 105 * mm
box(18 * mm, y - 62 * mm, W - 36 * mm, 62 * mm, 'Tools')
tools = ['soldering iron\n25-40 W + solder', 'small Phillips\nscrewdriver (PH1)', 'flat-nose pliers\n(for nuts)', 'masking / paper\ntape', 'helping hands or\nBlu-Tack (optional)']
tx = 30 * mm
for i, t in enumerate(tools):
    cx = tx + i * 34 * mm
    cy = y - 30 * mm
    c.setStrokeColor(INK); c.setLineWidth(1.2)
    if i == 0:
        c.setFillColor(colors.HexColor('#2d6cdf')); c.roundRect(cx - 3 * mm, cy - 2 * mm, 18 * mm, 4 * mm, 1 * mm, fill=1, stroke=1)
        c.setFillColor(GREY); c.rect(cx - 12 * mm, cy - 0.7 * mm, 9 * mm, 1.4 * mm, fill=1, stroke=0)
    elif i == 1:
        c.setFillColor(YEL); c.roundRect(cx, cy - 2.5 * mm, 12 * mm, 5 * mm, 1.5 * mm, fill=1, stroke=1)
        c.setFillColor(GREY); c.rect(cx - 10 * mm, cy - 0.6 * mm, 10 * mm, 1.2 * mm, fill=1, stroke=0)
    elif i == 2:
        c.setStrokeColor(RED); c.setLineWidth(3)
        c.line(cx - 6 * mm, cy - 6 * mm, cx + 2 * mm, cy + 1 * mm); c.line(cx + 6 * mm, cy - 6 * mm, cx - 2 * mm, cy + 1 * mm)
        c.setStrokeColor(INK); c.setLineWidth(2); c.line(cx - 2 * mm, cy + 1 * mm, cx - 1 * mm, cy + 7 * mm); c.line(cx + 2 * mm, cy + 1 * mm, cx + 1 * mm, cy + 7 * mm)
    elif i == 3:
        c.setFillColor(colors.HexColor('#f5e6b8')); c.circle(cx, cy, 6 * mm, fill=1, stroke=1)
        c.setFillColor(colors.white); c.circle(cx, cy, 3 * mm, fill=1, stroke=1)
    else:
        c.setFillColor(colors.HexColor('#5aa0d8')); c.ellipse(cx - 6 * mm, cy - 3 * mm, cx + 6 * mm, cy + 3 * mm, fill=1, stroke=1)
    for j, l in enumerate(t.split('\n')):
        txt(cx, y - 46 * mm - j * 11, l, 8, anchor='c')

# how to read
y = H - 175 * mm
box(18 * mm, y - 80 * mm, W - 36 * mm, 80 * mm, 'How to read this guide')
yy = y - 42
warn(28 * mm, yy - 2, 7, 'N3')
para(50 * mm, yy + 2, ['The official sheet does not say this clearly. What you see is our best', 'reading of the repo files. Read that note on p.12 first.'], 9, 12)
yy -= 36
txt(30 * mm, yy, '4x', 16, 'FB'); txt(50 * mm, yy + 2, 'Use this many of the part shown.', 9)
yy -= 30
arrow(26 * mm, yy + 5, 42 * mm, yy + 5); txt(50 * mm, yy + 2, 'Move / push the part this way.', 9)
yy -= 30
cross(33 * mm, yy + 5); txt(50 * mm, yy + 2, 'Wrong. Don\'t do this.', 9)
tick(83 * mm + 30 * mm, yy + 5); txt(125 * mm, yy + 2, 'Right.', 9)
yy -= 30
c.setFillColor(colors.HexColor('#ffe9a8')); c.rect(26 * mm, yy - 2, 16 * mm, 12, fill=1, stroke=0)
txt(34 * mm, yy + 1, '₹', 10, 'FB', anchor='c'); txt(50 * mm, yy + 2, 'India tip: where to get it locally, rough price.', 9)
newpage()

# =============================== PAGE 3: parts ===============================
header('Parts in the box', 'Check you have everything before step 1. Letters are used in the steps.')
cells = []
gx, gy = 18 * mm, H - 38 * mm
cw, ch = (W - 36 * mm - 8 * mm) / 3, 58 * mm


def cell(i, letter, name, qty, draw, note=None):
    col, row = i % 3, i // 3
    x = gx + col * (cw + 4 * mm)
    y = gy - row * (ch + 4 * mm)
    box(x, y - ch, cw, ch)
    txt(x + 8, y - 16, letter, 14, 'FB')
    txt(x + cw - 8, y - ch + 10, qty, 15, 'FB', anchor='r')
    lines = name.split('\n')
    for j, l in enumerate(lines):
        txt(x + 8, y - ch + 22 - j * 10 + (len(lines) - 1) * 10, l, 7.5)
    draw(x + cw / 2, y - ch / 2 + 4 * mm)
    if note:
        warn(x + cw - 26, y - 20, 6, note)


def d_board(cx, cy):
    board_top(cx - 52.5 * 1.5, cy + 16.5 * 1.5, 1.5, enc=False, fan_hdr=False)


def d_fan(cx, cy):
    fan_top(cx - 18 * 2.6 / 2 - 10, cy + 15 * 2.6 / 2 + 15, 1.5 * 1.3)


def d_enc(cx, cy):
    s = 3.0
    c.setFillColor(colors.HexColor('#777777')); c.setStrokeColor(INK)
    c.rect(cx - 6 * s, cy - 6 * s, 12 * s, 6 * s, fill=1, stroke=1)
    c.setFillColor(colors.HexColor('#cccccc')); c.rect(cx - 3.5 * s, cy, 7 * s, 3 * s, fill=1, stroke=1)
    c.rect(cx - 3 * s, cy + 3 * s, 6 * s, 8 * s, fill=1, stroke=1)
    c.setFillColor(colors.HexColor('#d4af37'))
    for dx in (-2.5, 0, 2.5):
        c.rect(cx + dx * s - 0.4 * s, cy - 10 * s, 0.8 * s, 4 * s, fill=1, stroke=0)
    c.setFillColor(colors.HexColor('#999999'))
    for dx in (-5.7, 5.7):
        c.rect(cx + dx * s - 0.6 * s, cy - 10 * s, 1.2 * s, 4 * s, fill=1, stroke=0)


def d_hdr(cx, cy):
    s = 4
    c.setFillColor(colors.black); c.rect(cx - 2.5 * s, cy - 1.2 * s, 5 * s, 2.5 * s, fill=1, stroke=0)
    c.setFillColor(colors.HexColor('#d4af37'))
    for dx in (-1.27, 1.27):
        c.rect(cx + dx * s - 0.32 * s, cy - 4 * s, 0.64 * s, 10 * s, fill=1, stroke=0)


def d_knob(cx, cy):
    s = 3.2
    c.setFillColor(ALU); c.setStrokeColor(INK)
    c.circle(cx, cy, 7 * s, fill=1, stroke=1)
    c.setStrokeColor(GREY); c.circle(cx, cy, 5 * s, fill=0, stroke=1)
    c.setFillColor(INK); c.circle(cx, cy, 3 * s, fill=1, stroke=0)
    c.setFillColor(ALU); c.rect(cx - 3 * s, cy - 0.7 * s, 6 * s, 1.4 * s, fill=1, stroke=0)


def d_lid(cx, cy):
    s = 3.0
    c.setFillColor(colors.white); c.setStrokeColor(INK)
    c.circle(cx, cy, 10 * s, fill=1, stroke=1)
    import math
    c.setFillColor(LIGHT)
    for k in range(5):
        a = math.radians(90 + 72 * k)
        c.circle(cx + 5.6 * s * math.cos(a), cy + 5.6 * s * math.sin(a), 2.2 * s, fill=1, stroke=1)
    c.setFillColor(colors.white); c.circle(cx, cy, 1.6 * s, fill=1, stroke=1)


def d_spring(cx, cy):
    c.setStrokeColor(INK); c.setLineWidth(1.2)
    for i in range(8):
        c.ellipse(cx - 9, cy - 22 + i * 5, cx + 9, cy - 18 + i * 5, fill=0, stroke=1)


def d_thumb(cx, cy):
    screw_side(cx, cy + 22, 1.6, 25, head='thumb', col=colors.HexColor('#222222'))


def d_feet(cx, cy):
    for k in range(4):
        foot_side(cx - 30 + k * 20, cy - 3, 2.6)


cell(0, 'A', 'Main board with screen and\nheat block (pre-assembled)', '1x', d_board)
cell(1, 'B', '30 x 30 x 10 mm fan, 5 V,\n2 wires', '1x', d_fan)
cell(2, 'C', 'Rotary knob switch\n(EC11 encoder, 5 pins + 2 tabs)', '1x', d_enc)
cell(3, 'D', '2-pin header for the fan', '1x', d_hdr)
cell(4, 'E', 'Aluminium knob cap\n(6 mm shaft)', '1x', d_knob)
cell(5, 'F', 'Tube lid disc (white,\nwith holes)', '1x', d_lid)
cell(6, 'G', 'Small spring', '1x', d_spring)
cell(7, 'H', 'Black nylon thumb screw\nM3 x 25 mm', '1x', d_thumb)
cell(8, 'J', 'Rubber feet', '4x', d_feet)
txt(18 * mm, gy - 3 * (ch + 4 * mm) - 4 * mm, 'Also in the kit: mineral oil (for the samples). Screws and spacers are on the next page.', 9)
txt(18 * mm, gy - 3 * (ch + 4 * mm) - 9 * mm, 'Parts list from the official instructions and the repo bill of materials (PocketPCR_BOM.ods).', 8, col=GREY)
newpage()

# =============================== PAGE 4: hardware at actual size ===============================
header('Screws and spacers at ACTUAL SIZE', 'Lay each one on the page to check it. Print at 100 % ("actual size", not "fit to page").')
S = mm  # 1 mm = 1 mm on paper
y0 = H - 50 * mm
items = [
    ('K', 'Screw M3 x 16', '4x', lambda x, y: screw_side(x, y, S, 16)),
    ('L', 'Screw M3 x 6', '2x', lambda x, y: screw_side(x, y, S, 6)),
    ('M', 'Nut M3', '2x', lambda x, y: (nut_side(x, y, S), nut_top(x, y - 12 * mm, S))),
    ('N', 'Spacer M3 x 10\n(hex, metal)', '2x', lambda x, y: spacer_side(x, y, S, 10)),
    ('P', 'Spacer M3 x 20\n(hex, metal)', '2x', lambda x, y: spacer_side(x, y, S, 20)),
    ('Q', 'White nylon screw\nM3 x 6, flat head', '1x', lambda x, y: screw_side(x, y, S, 6, head='csk', col=colors.HexColor('#f4f4f4'))),
]
bw = (W - 36 * mm - 10 * mm) / 6
for i, (L, name, q, fn) in enumerate(items):
    x = 18 * mm + i * (bw + 2 * mm)
    box(x, y0 - 70 * mm, bw, 70 * mm)
    txt(x + 6, y0 - 14, L, 13, 'FB')
    txt(x + bw - 6, y0 - 14, q, 12, 'FB', anchor='r')
    fn(x + bw / 2, y0 - 12 * mm)
    for j, l in enumerate(name.split('\n')):
        txt(x + bw / 2, y0 - 60 * mm - j * 9, l, 7.2, anchor='c')
warn(18 * mm + 5 * (bw + 2 * mm) + 4, y0 - 52 * mm, 5, 'N2')

# ruler
ry = y0 - 85 * mm
c.setStrokeColor(INK); c.setLineWidth(0.6)
c.line(20 * mm, ry, 120 * mm, ry)
for i in range(0, 101):
    h = 4 * mm if i % 10 == 0 else (2.5 * mm if i % 5 == 0 else 1.5 * mm)
    c.line(20 * mm + i * mm, ry, 20 * mm + i * mm, ry + h)
    if i % 10 == 0:
        txt(20 * mm + i * mm, ry - 9, str(i), 6.5, anchor='c')
txt(125 * mm, ry, 'mm  - check: this ruler must measure 100 mm', 8)

y = ry - 20 * mm
box(18 * mm, y - 65 * mm, W - 36 * mm, 65 * mm, 'Where each one goes')
rows = [('K  M3 x 16  (4)', 'hold the fan under the heat block (step 3)'),
        ('M  nut  (2) + N  10 mm spacer  (2)', 'go on the ends of those 4 screws, under the fan (step 3)'),
        ('L  M3 x 6  (2) + P  20 mm spacer  (2)', 'legs at the USB end (step 5)'),
        ('J  rubber feet  (4)', 'push onto the 4 legs (step 6)'),
        ('H  thumb screw + F lid + G spring', 'tube lid (step 8)'),
        ('Q  white nylon M3 x 6', 'listed in the parts file, use not described (see N2)')]
yy = y - 32
for a, b in rows:
    txt(26 * mm, yy, a, 9, 'FB'); txt(100 * mm, yy, b, 9); yy -= 15
txt(18 * mm, y - 75 * mm, '₹  Lost a screw? M3 screws, nuts and brass/steel hex spacers are sold in assorted kits on Robu.in, Robocraze, Amazon.in', 8.5)
txt(18 * mm, y - 80 * mm, '     and at any hardware shop (ask for "M3 bolt" and "M3 hex standoff / PCB spacer").', 8.5)
newpage()


# =============================== STEPS ===============================
def step_frame(n, title, y_top, h):
    box(18 * mm, y_top - h, W - 36 * mm, h)
    step_badge(18 * mm + 22, y_top - 22, n)
    txt(18 * mm + 46, y_top - 27, title, 13, 'FB')


# ----- page 5: steps 1-2 (soldering)
header('Assembly  1-2', 'Soldering. Take your time: 2 joints in step 1, 5 + 2 in step 2.')
s = 2.1
# step 1
yt = H - 38 * mm
step_frame(1, 'Fan header D: under the board, solder on top', yt, 118 * mm)
# left: bottom view, header inserted from below (side view)
sx, sy = 30 * mm, yt - 55 * mm
c.setFillColor(PCB); c.rect(sx, sy, 55 * mm, 1.6 * mm * 1.6, fill=1, stroke=0)
txt(sx, sy + 8 * mm, 'side view', 8, col=GREY)
txt(sx + 56 * mm, sy + 1, 'board A', 8)
txt(sx + 56 * mm, sy + 7 * mm, 'TOP (screen side)', 7.5, col=GREY)
txt(sx + 56 * mm, sy - 8 * mm, 'BOTTOM', 7.5, col=GREY)
hx = sx + 22 * mm
c.setFillColor(colors.black); c.rect(hx - 3 * mm, sy - 3 * mm, 6 * mm, 2.5 * mm, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#d4af37'))
for dx in (-1.27, 1.27):
    c.rect(hx + dx * mm - 0.35 * mm, sy - 10 * mm, 0.7 * mm, 13.5 * mm, fill=1, stroke=0)
arrow(hx, sy - 22 * mm, hx, sy - 12 * mm)
txt(hx + 5 * mm, sy - 18 * mm, 'push in from BELOW', 8.5, 'FB')
txt(hx - 3 * mm, sy - 30 * mm, 'long pins point DOWN (fan plugs on there)', 8)
# solder blobs on top
c.setFillColor(colors.HexColor('#c0c0c0'))
for dx in (-1.27, 1.27):
    c.circle(hx + dx * mm, sy + 3.2 * mm, 1.0 * mm, fill=1, stroke=0)
arrow(hx + 14 * mm, sy + 12 * mm, hx + 2.5 * mm, sy + 4.2 * mm, col=ORANGE)
txt(hx + 15 * mm, sy + 12 * mm, 'solder from TOP', 8.5, 'FB', ORANGE)
# right: top view of board with header location
Pq = board_top(122 * mm, yt - 30 * mm, 0.6 * mm, enc=False)
fx, fy = Pq(39.2, 27.7)
c.setStrokeColor(RED); c.setLineWidth(2); c.circle(fx, fy, 4 * mm, fill=0, stroke=1)
txt(fx, fy - 9 * mm, 'J1: 2 holes between the', 8, anchor='c')
txt(fx, fy - 12.5 * mm, 'block and the screen, near the edge', 8, anchor='c')
# good/bad joints
gy0 = yt - 100 * mm
txt(30 * mm, gy0 + 10 * mm, 'Joints:', 9, 'FB')
for k, (ok, lab) in enumerate([(True, 'shiny cone'), (False, 'touching = short!'), (False, 'blob / not wetted')]):
    cx = 60 * mm + k * 38 * mm
    c.setFillColor(colors.HexColor('#c0c0c0'))
    if k == 0:
        c.circle(cx - 3 * mm, gy0, 1.6 * mm, fill=1, stroke=0); c.circle(cx + 3 * mm, gy0, 1.6 * mm, fill=1, stroke=0)
    elif k == 1:
        c.roundRect(cx - 5 * mm, gy0 - 1.6 * mm, 10 * mm, 3.2 * mm, 1.6 * mm, fill=1, stroke=0)
    else:
        c.circle(cx - 3 * mm, gy0 + 1 * mm, 2.6 * mm, fill=1, stroke=0); c.circle(cx + 3 * mm, gy0, 1.6 * mm, fill=1, stroke=0)
    (tick if ok else cross)(cx + 9 * mm, gy0 + 2 * mm, 5)
    txt(cx, gy0 - 7 * mm, lab, 8, anchor='c')
warn(W - 40 * mm, yt - 112 * mm, 6, 'N1')

# step 2
yt = H - 162 * mm
step_frame(2, 'Knob switch C: in from the TOP, solder underneath', yt, 108 * mm)
Pq = board_top(30 * mm, yt - 34 * mm, 0.8 * mm, enc=False)
ex, ey = Pq(81.9, 24.0)
c.setStrokeColor(RED); c.setLineWidth(2); c.circle(ex, ey, 9 * mm, fill=0, stroke=1)
arrow(ex, ey + 30 * mm, ex, ey + 12 * mm)
txt(ex + 4 * mm, ey + 25 * mm, 'C from above', 8.5, 'FB')
txt(30 * mm, yt - 70 * mm, 'Top view. 3 pins on one side, 2 on the other,', 8.5)
txt(30 * mm, yt - 74.5 * mm, 'plus 2 wide metal tabs. They only fit one way.', 8.5)
# side view with tape
sx, sy = 125 * mm, yt - 55 * mm
c.setFillColor(PCB); c.rect(sx, sy, 50 * mm, 2.5 * mm, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#777777')); c.rect(sx + 15 * mm, sy + 2.5 * mm, 12 * mm, 6 * mm, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#cccccc')); c.rect(sx + 18 * mm, sy + 8.5 * mm, 6 * mm, 11 * mm, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#f5e6b8')); c.rect(sx + 10 * mm, sy + 8 * mm, 22 * mm, 3 * mm, fill=1, stroke=0)
txt(sx + 33 * mm, sy + 9 * mm, 'tape holds it', 7.5)
txt(sx + 33 * mm, sy + 5.5 * mm, 'flat while you flip', 7.5)
c.setFillColor(colors.HexColor('#c0c0c0'))
for dx in (16, 19, 22, 25, 26.3):
    pass
for dx in (16.5, 20.5, 25.5):
    c.circle(sx + dx * mm, sy - 0.8 * mm, 1.1 * mm, fill=1, stroke=0)
arrow(sx + 21 * mm, sy - 14 * mm, sx + 21 * mm, sy - 3 * mm, col=ORANGE)
txt(sx + 6 * mm, sy - 19 * mm, 'flip board, solder all 5 pins', 8.5, 'FB', ORANGE)
txt(sx + 6 * mm, sy - 23.5 * mm, '+ both tabs from BELOW', 8.5, 'FB', ORANGE)
c.setFillColor(LIGHT); c.rect(30 * mm, yt - 100 * mm, W - 60 * mm, 16 * mm, fill=1, stroke=0)
txt(34 * mm, yt - 91 * mm, 'Check: knob sits flat, not tilted. Each pin has its own joint. Tabs soldered too.', 8.5)
txt(34 * mm, yt - 96 * mm, 'Don\'t heat one pin longer than ~3 seconds; let it cool and come back.', 8.5)
newpage()

# ----- page 6: steps 3-5
header('Assembly  3-5', 'Mechanical. Finger-tight + a small turn with pliers. Over-tightening cracks the plastic fan.')
yt = H - 38 * mm
step_frame(3, 'Fan B under the heat block: 4x K from the top', yt, 120 * mm)
# side/exploded view
s = 2.0 * mm / mm  # factor
S2 = 2.0  # pt per mm scale 2 -> mm*2? use mm-based
k = 1.3 * mm
bx0, by0 = 32 * mm, yt - 52 * mm
txt(bx0, yt - 15 * mm, 'side view, fan under the block', 8, col=GREY)
c.setFillColor(PCB); c.rect(bx0, by0, 45 * k, 1.6 * k, fill=1, stroke=0)
txt(bx0 + 46 * k, by0, 'A', 11, 'FB')
c.setFillColor(ALU); c.setStrokeColor(INK); c.rect(bx0 + 6 * k, by0 + 1.6 * k, 19.5 * k, 7 * k, fill=1, stroke=1)
fy0 = by0 - 6 * k
fan_side(bx0 + 1 * k, fy0, k)
txt(bx0 + 33 * k, fy0 - 7 * k, 'B', 11, 'FB')
for hx in (4, 28):
    screw_side(bx0 + hx * k, by0 + 24 * k, k, 16)
txt(bx0 + 34 * k, by0 + 17 * k, '4x K', 12, 'FB')
arrow(bx0 + 16 * k, by0 + 22 * k, bx0 + 16 * k, by0 + 12 * k, w=1.8, head=6)
nut_side(bx0 + 4 * k, fy0 - 15 * k, k)
spacer_side(bx0 + 28 * k, fy0 - 14 * k, k, 10)
arrow(bx0 + 16 * k, fy0 - 26 * k, bx0 + 16 * k, fy0 - 13 * k, w=1.8, head=6)
txt(bx0 + 4 * k, fy0 - 23 * k, 'M', 10, 'FB', anchor='c')
txt(bx0 + 33 * k, fy0 - 20 * k, 'N', 10, 'FB')
txt(bx0, fy0 - 33 * k, '2x M on two screws,  2x N on the other two', 9, 'FB')
warn(bx0 + 70 * k, fy0 - 34 * k, 5, 'N3')
# top view right: cable direction
fx0, fy1 = 125 * mm, yt - 30 * mm
fan_top(fx0, fy1, 1.25 * mm)
txt(fx0, fy1 - 48 * mm, 'Seen from below: the fan cable must', 8.5)
txt(fx0, fy1 - 52.5 * mm, 'come out on the side of the header D.', 8.5)
arrow(fx0 + 50 * mm, fy1 - 25 * mm, fx0 + 50 * mm, fy1 - 33 * mm, col=RED)
txt(fx0 + 46 * mm, fy1 - 22 * mm, 'to D', 8.5, 'FB', RED)
txt(fx0, fy1 - 60 * mm, 'Label/sticker side of the fan faces the', 8.5)
txt(fx0, fy1 - 64.5 * mm, 'board, so it blows UP onto the block.', 8.5)
warn(fx0 + 56 * mm, fy1 - 66 * mm, 5, 'N4')
# step 4
yt = H - 163 * mm
step_frame(4, 'Plug the fan cable onto header D', yt, 42 * mm)
cx, cy = 60 * mm, yt - 28 * mm
c.setFillColor(colors.white); c.setStrokeColor(INK); c.rect(cx - 4 * mm, cy - 3 * mm, 8 * mm, 6 * mm, fill=1, stroke=1)
c.setStrokeColor(RED); c.setLineWidth(1.4); c.line(cx - 30 * mm, cy + 0.8 * mm, cx - 4 * mm, cy + 0.8 * mm)
c.setStrokeColor(colors.black); c.line(cx - 30 * mm, cy - 0.8 * mm, cx - 4 * mm, cy - 0.8 * mm)
c.setFillColor(colors.HexColor('#d4af37'))
c.rect(cx + 8 * mm, cy - 2 * mm, 7 * mm, 0.8 * mm, fill=1, stroke=0); c.rect(cx + 8 * mm, cy + 1 * mm, 7 * mm, 0.8 * mm, fill=1, stroke=0)
arrow(cx + 6 * mm, cy + 7 * mm, cx + 12 * mm, cy + 7 * mm)
para(100 * mm, yt - 20 * mm, ['Push it fully on. If the fan doesn\'t spin when', 'cooling later, turn the plug round 180°.'], 9, 12)
warn(100 * mm, yt - 40 * mm, 5, 'N4')
# step 5
yt = H - 210 * mm
step_frame(5, 'Two legs at the USB end: 2x L + 2x P', yt, 62 * mm)
bx0, by0 = 40 * mm, yt - 26 * mm
c.setFillColor(PCB); c.rect(bx0, by0, 40 * k, 1.6 * k, fill=1, stroke=0)
txt(bx0 + 41 * k, by0, 'A, USB end', 8.5, 'FB')
screw_side(bx0 + 30 * k, by0 + 11 * k, k, 6)
spacer_side(bx0 + 30 * k, by0 - 2 * k, k, 20)
txt(bx0 + 34 * k, by0 + 6 * k, 'L', 10, 'FB'); txt(bx0 + 34 * k, by0 - 14 * k, 'P', 10, 'FB')
arrow(bx0 + 22 * k, by0 + 10 * k, bx0 + 22 * k, by0 + 3 * k, w=1.6, head=5)
arrow(bx0 + 22 * k, by0 - 20 * k, bx0 + 22 * k, by0 - 4 * k, w=1.6, head=5)
txt(bx0 + 5 * k, by0 - 12 * k, '2x', 14, 'FB')
para(120 * mm, yt - 20 * mm, ['Screw L down through the board into', 'the 20 mm spacer P. Both legs (fan end and', 'USB end) now end at the same height', '(10 mm fan + 10 mm spacer = 20 mm).'], 9, 12)
warn(120 * mm, yt - 58 * mm, 5, 'N3')
newpage()

# ----- page 7: steps 6-10
header('Assembly  6-10')
yt = H - 34 * mm
step_frame(6, 'Rubber feet J on all 4 legs', yt, 36 * mm)
for i in range(4):
    x = 80 * mm + i * 25 * mm
    spacer_side(x, yt - 12 * mm, 1.3 * mm, 8)
    foot_side(x, yt - 30 * mm, 1.3 * mm)
    arrow(x, yt - 34 * mm, x, yt - 31 * mm, w=1.4, head=4)
txt(40 * mm, yt - 30 * mm, '4x J', 13, 'FB')

yt -= 41 * mm
step_frame(7, 'Push knob cap E onto the switch shaft', yt, 36 * mm)
d_knob(80 * mm, yt - 22 * mm)
arrow(105 * mm, yt - 22 * mm, 92 * mm, yt - 22 * mm)
para(115 * mm, yt - 18 * mm, ['By hand, straight down. Match the flat side of', 'the shaft. Leave a small gap so it can click.'], 9, 12)

yt -= 41 * mm
step_frame(8, 'Tube lid: H through F, spring G underneath', yt, 60 * mm)
cx, cy = 70 * mm, yt - 20 * mm
screw_side(cx, cy, 1.4 * mm, 25, head='thumb', col=colors.HexColor('#222222'))
c.setFillColor(colors.white); c.setStrokeColor(INK); c.rect(cx - 14 * mm, cy - 12 * mm, 28 * mm, 1.7 * mm, fill=1, stroke=1)
txt(cx + 16 * mm, cy - 12 * mm, 'F', 10, 'FB')
c.setLineWidth(0.9)
for i in range(5):
    c.ellipse(cx - 3 * mm, cy - 18 * mm - i * 2 * mm, cx + 3 * mm, cy - 16.5 * mm - i * 2 * mm, fill=0, stroke=1)
txt(cx + 6 * mm, cy - 22 * mm, 'G', 10, 'FB')
txt(cx - 26 * mm, cy - 5 * mm, 'H', 10, 'FB')
para(110 * mm, yt - 18 * mm, ['Screw H goes down through the middle hole of', 'disc F, then through spring G, and later into the', 'threaded hole in the centre of the heat block.', 'The spring presses the tubes onto the block.'], 9, 12)
warn(110 * mm, yt - 58 * mm, 5, 'N5')

yt -= 65 * mm
step_frame(9, 'Optional: 3D-printed case', yt, 50 * mm)
para(30 * mm, yt - 34 * mm + 12 * mm, ['Free file: thingiverse.com/thing:4094823 (any colour, PLA/PETG).',
                                          'a) pull the 4 rubber feet off  b) press the board into the case firmly',
                                          'c) push the feet back on from under the case.',
                                          '₹  No printer? Local 3D-print shops / makerspaces, or online: Robu 3D print service,',
                                          '    Karkhana.io, IndiaMART "3D printing service" listings. Roughly ₹200-500 for this size.'], 9, 12.5)
warn(W - 40 * mm, yt - 46 * mm, 5, 'N8')

yt -= 55 * mm
step_frame(10, 'First power-on', yt, 40 * mm)
para(30 * mm, yt - 24 * mm, ['Plug in USB-C (5 V, 2 A or more). The screen shows the software version, then',
                              '"Run PCR / Setup". Turn the knob: the highlight moves. Done!',
                              'Left-handed? Hold the knob pressed while plugging in: the screen turns upside down.'], 9, 12.5)
newpage()

# =============================== PAGE 8: loading samples & running ===============================
header('Load samples and run', 'From the official instructions.')
yt = H - 38 * mm
box(18 * mm, yt - 78 * mm, W - 36 * mm, 78 * mm, 'Prepare the tubes')
# tube drawing
for i, (lab, col, hh) in enumerate([('25-50 µl\nreaction', colors.HexColor('#7fb3ff'), 14), ('15-30 µl\nmineral oil', colors.HexColor('#f5e08a'), 6)]):
    pass
tx, ty = 45 * mm, yt - 65 * mm
p = c.beginPath(); p.moveTo(tx - 5 * mm, ty + 40 * mm); p.lineTo(tx + 5 * mm, ty + 40 * mm); p.lineTo(tx + 3 * mm, ty + 8 * mm); p.lineTo(tx, ty); p.lineTo(tx - 3 * mm, ty + 8 * mm); p.close()
c.setFillColor(colors.white); c.setStrokeColor(INK); c.drawPath(p, fill=1, stroke=1)
p = c.beginPath(); p.moveTo(tx - 3.7 * mm, ty + 18 * mm); p.lineTo(tx + 3.7 * mm, ty + 18 * mm); p.lineTo(tx + 3 * mm, ty + 8 * mm); p.lineTo(tx, ty); p.lineTo(tx - 3 * mm, ty + 8 * mm); p.close()
c.setFillColor(colors.HexColor('#7fb3ff')); c.drawPath(p, fill=1, stroke=0)
c.setFillColor(colors.HexColor('#f5e08a')); c.rect(tx - 4.2 * mm, ty + 18 * mm, 8.4 * mm, 5 * mm, fill=1, stroke=0)
txt(tx + 8 * mm, ty + 20 * mm, 'oil 15-30 µl', 8.5, 'FB'); txt(tx + 8 * mm, ty + 12 * mm, 'sample 25-50 µl', 8.5, 'FB')
para(100 * mm, yt - 26 * mm, ['1  Use standard 0.2 ml PCR tubes.', '2  Pipette 25-50 µl of reaction.', '3  Add 15-30 µl mineral oil on top', '    (there is no heated lid; oil stops', '    the sample boiling away).', '4  Put at least 3 tubes in the block', '    (fill empty spots with blank tubes).', '5  Lid on: screw H in gently, no force.'], 9, 12.5)
txt(22 * mm, yt - 74 * mm, '₹  Mineral oil substitute: "Liquid Paraffin IP / Light Liquid Paraffin" from a chemist. See N6.', 8.5)

yt = H - 125 * mm
box(18 * mm, yt - 70 * mm, W - 36 * mm, 70 * mm, 'Factory program (change it in Setup)')
hdr = ['Step', 'Init', 'Denature', 'Annealing', 'Extension', 'Final']
vals = [['Temp °C', '94', '94', '55', '72', '25'], ['Time', '60 s', '120 s', '80 s', '40 s', '3 min']]
x0 = 26 * mm
cwid = 27 * mm
for j, h_ in enumerate(hdr):
    c.setFillColor(INK if j else colors.HexColor('#555555')); c.rect(x0 + j * cwid, yt - 30 * mm, cwid - 1, 8 * mm, fill=1, stroke=0)
    txt(x0 + j * cwid + cwid / 2, yt - 27.5 * mm, h_, 9, 'FB', colors.white, 'c')
for i, row in enumerate(vals):
    for j, v in enumerate(row):
        c.setFillColor(LIGHT if i % 2 == 0 else colors.white); c.rect(x0 + j * cwid, yt - 38 * mm - i * 8 * mm, cwid - 1, 8 * mm, fill=1, stroke=0)
        txt(x0 + j * cwid + cwid / 2, yt - 35.5 * mm - i * 8 * mm, v, 9, 'FB' if j == 0 else 'F', anchor='c')
txt(26 * mm, yt - 60 * mm, 'Denature > Annealing > Extension repeat 25x (cycles). These are the firmware defaults; your kit may differ.', 8.5)
txt(26 * mm, yt - 65 * mm, 'Times over 90 s are set in minutes. Temperatures 25-99 °C, cycles 1-99.', 8.5)
warn(W - 42 * mm, yt - 20 * mm, 5, 'N7')

yt = H - 205 * mm
box(18 * mm, yt - 55 * mm, W - 36 * mm, 55 * mm, 'Run')
para(26 * mm, yt - 34, ['1  Main screen: turn to "Run PCR", press. The block heats up.', '2  Screen shows step, cycle (e.g. 3 of 25), set temp, block temp, seconds left.',
                       '3  To stop early: turn the knob until "Press to STOP PCR", then press.', '4  "PCR Done": press to go back. Let the block cool before taking tubes out.',
                       '5  Incubator trick: set all temperatures the same; time x cycles = total time.'], 9, 13)
newpage()

# =============================== PAGE 9: screens map ===============================
header('Screens and knob map', 'Turn = move / change value.   Press = select / confirm.')


def oled(x, y, lines, inv=None, w=62 * mm, h=31 * mm):
    c.setFillColor(colors.black); c.roundRect(x, y - h, w, h, 2 * mm, fill=1, stroke=0)
    for i, l in enumerate(lines):
        yy = y - 7 * mm - i * 5.5 * mm
        if inv is not None and i == inv:
            c.setFillColor(colors.HexColor('#7fd3ff')); c.rect(x + 3 * mm, yy - 1.5 * mm, w - 6 * mm, 5.5 * mm, fill=1, stroke=0)
            txt(x + 5 * mm, yy, l, 8.5, 'FB', colors.black)
        else:
            txt(x + 5 * mm, yy, l, 8.5, 'F', colors.HexColor('#7fd3ff'))


yt = H - 40 * mm
oled(22 * mm, yt, ['', 'Run PCR', '', 'Setup'], inv=1)
txt(53 * mm, yt - 36 * mm, 'MAIN', 10, 'FB', anchor='c')
arrow(88 * mm, yt - 12 * mm, 112 * mm, yt - 12 * mm); txt(92 * mm, yt - 9 * mm, 'press', 8)
oled(116 * mm, yt, ['  PCR Running', 'State: Denature', 'Cycle: 3 of 25', 'Block: 93.8 °C  Time: 12 s'])
txt(147 * mm, yt - 36 * mm, 'RUN  (turn, then press = stop)', 9, 'FB', anchor='c')
arrow(53 * mm, yt - 40 * mm, 53 * mm, yt - 54 * mm); txt(56 * mm, yt - 48 * mm, 'turn to Setup, press', 8)
yt2 = yt - 58 * mm
oled(22 * mm, yt2, ['Denature Temp [°C]', '94  94  55  72  25', '      25x', '60 2m 80 40 3m'], inv=1)
txt(53 * mm, yt2 - 36 * mm, 'SETUP', 10, 'FB', anchor='c')
para(92 * mm, yt2 - 4 * mm, ['Turn: walk through the 11 values', '   (5 temperatures, 5 times, cycles).', 'Press: edit the value (box outline).', 'Turn: change it. Press: done.',
                              'Keep turning past the last value', '   to reach  Set / Cancel :', '   Set = save to memory,', '   Cancel = forget all changes.'], 9, 12.5)
yt3 = yt2 - 60 * mm
oled(22 * mm, yt3, ['   Set     Cancel', '', '', ''], inv=0, h=16 * mm)
oled(100 * mm, yt3, ['SENSOR ERROR', 'Heater switched off.', 'Unplug USB to reset.'], h=24 * mm)
txt(131 * mm, yt3 - 29 * mm, 'Only with the fixed firmware (see p.10)', 8, anchor='c')
newpage()

# =============================== PAGE 10: troubleshooting ===============================
header('Problems and fixes')
rows = [
    ('Screen stays dark / resets', 'Charger or cable too weak. Use 5 V 2 A+ and a short (<2 m) USB-C cable. Laptop ports often give only 0.5-0.9 A.'),
    ('Run never says "PCR Done"', 'In a warm room (above ~25 °C) the block can\'t cool to the 25 °C Final step, so it waits forever. Set Final to 35-40 °C, or update to the fixed firmware (it moves on after 2 min).'),
    ('Power cut in the middle', 'The run is lost; there is no resume. Run from a power bank (10,000 mAh, 2 A out) or a UPS-backed charger. A run of ~1 h uses ~5-8 Wh (estimate).'),
    ('Samples dried / volume low', 'More oil (30 µl), lid screwed down, no fan or AC blowing at the device.'),
    ('Tubes not at set temperature', 'Sensor sits in the block. Drafts or a cold room cool the tubes. Shield it; at least 3 tubes in.'),
    ('Second run only did 1 cycle', 'Known bug in firmware V1.02. Unplug and re-plug before each run, or flash the fix.'),
    ('Fan never spins', 'Plug reversed (step 4) or a header joint not soldered (step 1).'),
    ('Knob skips or counts wrong', 'Re-heat the 5 encoder joints (step 2).'),
    ('"SENSOR ERROR"', 'Fixed firmware only: the block reads above 110 °C or below -20 °C. Check the heater cable/connector at the block end.'),
]
y = H - 40 * mm
for a, b in rows:
    import textwrap
    lines = textwrap.wrap(b, 70)
    hh = max(1, len(lines)) * 11.5 + 12
    c.setFillColor(LIGHT); c.rect(18 * mm, y - hh, 58 * mm, hh, fill=1, stroke=0)
    txt(21 * mm, y - 15, a if len(a) < 30 else a[:30], 8.8, 'FB')
    para(80 * mm, y - 15, lines, 8.8, 11.5)
    y -= hh + 3 * mm
y -= 4 * mm
box(18 * mm, y - 52 * mm, W - 36 * mm, 52 * mm, 'Firmware fixes (your fork)')
para(26 * mm, y - 36, ['github.com/Amit-1of1/PocketPCR  -  pull request #1 (V1.03). Fixes: 2nd run doing 1 cycle; heater cut-off',
                       'if the sensor fails; hang on the Final step in warm rooms; stop prompt when turning left; and more.',
                       'To flash: Arduino IDE + SAMD21 board support, open code/PocketPCR_V2_quite_dev_i_disp, install',
                       'Adafruit GFX + SSD1306 libraries, connect USB, Upload. Not yet tested on a real device.'], 8.8, 12)
warn(W - 40 * mm, y - 50 * mm, 5, 'N9')
newpage()

# =============================== PAGE 11: India ===============================
header('Buying it in India', 'Rough prices, Nov 2026. Check before you buy; prices move a lot.')
rows = [
    ('What', 'Where in India', 'Rough ₹'),
    ('PocketPCR kit', 'Not stocked locally that we found: order from gaudishop.ch (Switzerland). Add\nshipping + customs duty/IGST on arrival (often ~30-40 % of value, see N10).', 'check site'),
    ('USB-C charger 5 V 2 A+', 'Any BIS-marked phone charger (Mi, Samsung, Ambrane, Portronics); Amazon.in,\nFlipkart, local mobile shops. Avoid unbranded ones: weak 2 A.', '300-700'),
    ('USB-C cable < 1 m', 'Same shops. Short and thick is better.', '150-300'),
    ('Power bank (backup)', '10,000 mAh, 2 A+ output. Rides through power cuts.', '800-1,500'),
    ('30x30x10 5 V fan', 'Robu.in, Robocraze, Amazon.in, SP Road (Bengaluru), Lamington Rd\n(Mumbai), Lajpat Rai Mkt (Delhi), Ritchie St (Chennai). Ask for "3010 5V".', '60-150'),
    ('EC11 rotary encoder', 'Robu.in, Robocraze, Electronicscomp, same markets.', '30-80'),
    ('M3 screws, nuts, hex spacers', 'Assorted kit, Robu / Amazon.in or hardware shop.', '150-400 kit'),
    ('Soldering iron 25-60 W + solder', 'Soldron (Indian brand) at any electronics market / Amazon.in.', '300-900'),
    ('0.2 ml PCR tubes', 'Tarsons (Kolkata), HiMedia, Genaxy; lab suppliers on IndiaMART.', '400-1,200 /\n1000'),
    ('Mineral oil', 'PCR-grade from HiMedia / Sigma India; or "Light Liquid Paraffin IP" from a\nchemist for hobby use (N6).', '80-150'),
    ('Taq master mix, primers', 'HiMedia, GeNei (Merck), Takara India, Bangalore Genei; primers from\nEurofins/Barcode Biosciences/Sigma India (2-4 day delivery).', 'varies'),
    ('Case print', 'Local 3D-print shop / makerspace, or online services.', '200-500'),
]
y = H - 40 * mm
colw = [46 * mm, 104 * mm, 24 * mm]
import textwrap
for i, r_ in enumerate(rows):
    cols_ = [textwrap.wrap(r_[0].replace('\n', ' '), 24), textwrap.wrap(r_[1].replace('\n', ' '), 64), r_[2].split('\n')]
    nl = max(len(x) for x in cols_)
    hh = nl * 10.5 + 8
    c.setFillColor(INK if i == 0 else (LIGHT if i % 2 else colors.white))
    c.rect(18 * mm, y - hh, sum(colw), hh, fill=1, stroke=0)
    x = 18 * mm
    for j, cl in enumerate(cols_):
        for li, l in enumerate(cl):
            txt(x + 3, y - 12 - li * 10.5, l, 8.2, 'FB' if (i == 0 or j == 0) else 'F', colors.white if i == 0 else INK)
        x += colw[j]
    y -= hh
y -= 6 * mm
box(18 * mm, y - 62 * mm, W - 36 * mm, 62 * mm, 'India-specific tips')
para(26 * mm, y - 36, ['Heat: rooms at 30-40 °C in summer. Set Final ≥ room + 5 °C (e.g. 40 °C), or flash the fixed firmware.',
                       '          Cooling to annealing (55 °C) still works but each cycle is slower in a hot room.',
                       'Power cuts: run from a power bank or a UPS-backed socket. A cut = start again with fresh tubes.',
                       'Fans / AC: keep ceiling fans and AC vents off the device; drafts cool the tubes (official tip).',
                       'Reagents: Taq mix must stay frozen; ask suppliers to ship with ice packs, store at -20 °C.',
                       'Mains: Indian 230 V / 50 Hz is fine; the device only ever sees 5 V from the charger.'], 8.8, 12.5)
newpage()

# =============================== PAGE 12: notes ===============================
header('Notes for the ⚠ flags', 'Where the official sheet is unclear and we filled the gap.')
notes = [
    ('N1', 'Fan header side: official text says connector on the BOTTOM, soldered from the TOP. The exact spot is from the KiCad layout (J1, 2-pin, 2.54 mm). Match the photo in the official PDF if it differs.'),
    ('N2', 'The parts file lists one white nylon M3 x 6 flat-head screw, but the instructions never use it. Possibly a spare or holds the heater board; keep it.'),
    ('N3', 'Which spacer/nut goes on which fan screw, and the leg holes at the USB end, are inferred from the parts file (2x 10 mm + 2x 20 mm spacers, 4x M3x16 + 2x M3x6) and the 10 mm fan thickness so all 4 legs end at 20 mm. Check against the photos in the official PDF.'),
    ('N4', 'Fan airflow direction and plug polarity are not stated. The fan should blow onto the block. Our reading: label towards the board. If it does not cool, flip the fan or the plug.'),
    ('N5', 'The lid stack order (screw through disc, then spring) is our reading of "disc, spring and long plastic screw". The heater drawing shows an M3 thread in the block centre.'),
    ('N6', 'Light liquid paraffin (IP) is mineral oil but not certified nuclease-free. Fine for hobby PCR; use PCR-grade oil for anything that matters.'),
    ('N7', 'Program values are the firmware defaults when memory is blank. A kit that was tested may ship with other values. The kit check (Setup) shows what yours has.'),
    ('N8', 'Case fit and feet order are from the official text; we have not printed the case.'),
    ('N9', 'Firmware changes were checked by compiling against stand-in libraries, not on a real PocketPCR. Test with water-filled tubes first.'),
    ('N10', 'Customs: imports by courier usually pay basic duty + IGST + handling. The total varies by item and rules; ask the courier for a quote.'),
]
y = H - 40 * mm
import textwrap
for n, t in notes:
    warn(18 * mm, y - 8, 5)
    txt(31 * mm, y - 6, n, 9, 'FB')
    lines = textwrap.wrap(t, 86)
    para(45 * mm, y - 6, lines, 8.6, 11)
    y -= len(lines) * 11 + 9
y -= 6 * mm
para(18 * mm, y, textwrap.wrap('Sources: official PocketPCR instructions by Urs Gaudenz / GaudiLabs, CC BY 4.0 copy (Instrucciones para la PocketPCR, N. A. Mendez & U. Gaudenz, 2023); github.com/GaudiLabs/PocketPCR (KiCad layout, BOM, firmware, GPL-3.0). gaudi.ch could not be reached when this was made, so the photos in the original PDF were not seen. This is an unofficial guide.', 104), 7.8, 10.5, col=colors.HexColor('#555555'))
newpage()
c.save()
print('ok', page_no[0])
