"""Reproducible nominal reference drawing; not a CAD-derived manufacturing drawing."""
from pathlib import Path
from math import sqrt
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.units import mm

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'output' / 'pdf' / 'T001_test_link_three_views_r01.pdf'
LENGTH, HEIGHT, THICKNESS = 180.0, 30.0, 8.0
PITCH, END_MARGIN = 150.0, 15.0
PIVOT_D, TIP_D = 12.0, 6.0
assert LENGTH == PITCH + 2 * END_MARGIN
assert PIVOT_D < HEIGHT and TIP_D < HEIGHT
assert END_MARGIN > max(PIVOT_D, TIP_D) / 2
pdfmetrics.registerFont(TTFont('Chinese', r'C:\Windows\Fonts\simhei.ttf'))
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
c = canvas.Canvas(str(OUTPUT), pagesize=(297 * mm, 210 * mm))
c.setTitle('T001 导入测试连杆：带尺寸三视图 r01')
c.setAuthor('AI辅助规格出图；原生SolidWorks建模：张轩瑞')

def text(x, y, value, size=9, align='left'):
    value = value.replace('Ø', 'φ')  # SimHei supports Greek phi, not Latin Ø.
    c.setFont('Chinese', size)
    getattr(c, {'left': 'drawString', 'center': 'drawCentredString', 'right': 'drawRightString'}[align])(x * mm, y * mm, value)

def line(x1, y1, x2, y2, width=0.35):
    c.setLineWidth(width)
    c.line(x1 * mm, y1 * mm, x2 * mm, y2 * mm)

def rect(x, y, w, h):
    c.setLineWidth(0.8)
    c.rect(x * mm, y * mm, w * mm, h * mm)

def arrow(x, y, dx, dy):
    norm = sqrt(dx * dx + dy * dy)
    dx, dy = dx / norm, dy / norm
    p = c.beginPath()
    p.moveTo(x * mm, y * mm)
    p.lineTo((x + 2.6 * dx + .65 * dy) * mm, (y + 2.6 * dy - .65 * dx) * mm)
    p.lineTo((x + 2.6 * dx - .65 * dy) * mm, (y + 2.6 * dy + .65 * dx) * mm)
    p.close()
    c.drawPath(p, fill=1, stroke=0)

def hdim(x1, x2, source_y, y, label, outside=False):
    sign = 1 if y > source_y else -1
    for x in (x1, x2):
        line(x, source_y + sign * 1.5, x, y + sign * 2)
    if outside:
        line(x1 - 6, y, x2 + 6, y)
        arrow(x1, y, -1, 0)
        arrow(x2, y, 1, 0)
    else:
        line(x1, y, x2, y)
        arrow(x1, y, 1, 0)
        arrow(x2, y, -1, 0)
    text((x1 + x2) / 2, y + 1.6, label, align='center')

def vdim(y1, y2, source_x, x, label):
    sign = 1 if x > source_x else -1
    for y in (y1, y2):
        line(source_x + sign * 1.5, y, x + sign * 2, y)
    line(x, y1, x, y2)
    arrow(x, y1, 0, 1)
    arrow(x, y2, 0, -1)
    c.saveState()
    c.translate((x - 1.6) * mm, (y1 + y2) / 2 * mm)
    c.rotate(90)
    c.setFont('Chinese', 9)
    c.drawCentredString(0, 0, label)
    c.restoreState()

def centerline(x1, y1, x2, y2):
    c.setDash([7 * mm, 1.5 * mm, .8 * mm, 1.5 * mm])
    line(x1, y1, x2, y2, .3)
    c.setDash()

def hidden(x1, y1, x2, y2):
    c.setDash([2 * mm, 1 * mm])
    line(x1, y1, x2, y2, .4)
    c.setDash()

def leader(x, y, elbow_x, elbow_y, end_x, label):
    line(x, y, elbow_x, elbow_y)
    line(elbow_x, elbow_y, end_x, elbow_y)
    arrow(x, y, elbow_x - x, elbow_y - y)
    text(elbow_x + 1, elbow_y + 1.7, label)

text(14, 196, 'T001 导入测试连杆 — 带尺寸三视图', 16)
text(14, 189, '单位：mm  |  比例：1:1  |  第一角法：俯视图在下，左视图在右', 10)
line(14, 185, 283, 185, .6)

# Front: sketch coordinates map (0,0) to (60,137) on paper.
rect(45, 122, LENGTH, HEIGHT)
for cx, diameter in ((60, PIVOT_D), (210, TIP_D)):
    c.setLineWidth(.8)
    c.circle(cx * mm, 137 * mm, diameter / 2 * mm)
    centerline(cx, 118, cx, 156)
centerline(41, 137, 229, 137)
hdim(45, 225, 152, 177, '180')
hdim(60, 210, 137, 166, '150')
vdim(122, 152, 45, 26, '30')
vdim(122, 137, 45, 36, '15')
hdim(45, 60, 122, 109, '15')
leader(60 + 6 / sqrt(2), 137 - 6 / sqrt(2), 88, 101, 118, 'Ø12 贯穿')
leader(210 + 3 / sqrt(2), 137 - 3 / sqrt(2), 234, 101, 267, 'Ø6 贯穿')
text(135, 115, '主视图', 10, 'center')

# Top view: thickness is centred about sketch plane; holes shown hidden.
rect(45, 76, LENGTH, THICKNESS)
for cx, diameter in ((60, PIVOT_D), (210, TIP_D)):
    for x in (cx - diameter / 2, cx + diameter / 2):
        hidden(x, 76, x, 84)
    centerline(cx, 73, cx, 87)
centerline(41, 80, 229, 80)
# Small dimension uses external arrows and horizontal label for readability.
line(32, 76, 43.5, 76)
line(32, 84, 43.5, 84)
line(34, 71, 34, 89)
arrow(34, 76, 0, -1)
arrow(34, 84, 0, 1)
text(29, 78.8, '8', align='right')
text(135, 68, '俯视图', 10, 'center')

# First-angle left view, placed to right of front.
rect(255, 122, THICKNESS, HEIGHT)
for diameter in (PIVOT_D, TIP_D):
    for y in (137 - diameter / 2, 137 + diameter / 2):
        hidden(255, y, 263, y)
centerline(251, 137, 267, 137)
centerline(259, 118, 259, 156)
hdim(255, 263, 152, 166, '8', outside=True)
text(259, 115, '左视图', 10, 'center')

text(14, 55, '建模基准：左侧 Ø12 孔心为原点；X 向右、Y 向上；厚度中面 Z=0，向两侧各 4 mm。', 9)
text(14, 49, '两孔均贯穿。外框 x=[-15,165]、y=[-15,15]；右孔中心 (150,0)。不加倒角、圆角或开槽。', 9)
text(14, 43, '右端余量 15 mm 由 180-150-15 得到；不要再添加重复驱动尺寸。材料赋值与质量检查待建模后完成。', 9)
text(14, 37, '用途：仅验证 CAD 导入；不是最终承载连杆。全部为名义尺寸，无加工公差；Ø6 不代表 M6 螺纹。', 9)
line(14, 31, 283, 31, .6)
text(14, 24, '图号 T001-REF-001  |  版本 r01  |  日期 2026-10-01  |  来源 S008', 9)
text(14, 18, 'AI 辅助规格出图；原生 SolidWorks 建模：张轩瑞。未由 CAD 关联生成，不替代 .SLDDRW。', 9)
text(283, 10, '建模参考图 · 非加工图', 11, 'right')
c.showPage()
c.save()
print(OUTPUT)
print('Checked: 180 = 15 + 150 + 15; envelope 180 x 30 x 8 mm; through holes 12/6 mm.')
