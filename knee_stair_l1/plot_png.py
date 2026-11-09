"""Small line-chart PNG writer. Uses only the standard library."""

from __future__ import annotations

import struct
import zlib


def _chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


_GLYPHS = {
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}


def _text(pix, x, y, text, rgb):
    cursor = x
    for char in text:
        glyph = _GLYPHS.get(char, _GLYPHS[" "])
        for row, bits in enumerate(glyph):
            for col, bit in enumerate(bits):
                if bit == "1":
                    pix(cursor + col, y + row, rgb)
        cursor += 6


def write_delay_plot(path, delays_ms: list[float], series: list[tuple[str, list[float], tuple[int, int, int]]]) -> None:
    """Two-row chart. series items are (name, values, rgb) and share delays_ms.

    The first two series are drawn on the top panel and the rest on the bottom.
    """
    width, height = 880, 640
    left, right, top = 70, 20, 36
    panel_h = 250
    gap = 50
    plot_w = width - left - right
    canvas = bytearray([255, 255, 255]) * (width * height)

    def pix(x, y, rgb):
        if 0 <= x < width and 0 <= y < height:
            i = (y * width + x) * 3
            canvas[i:i + 3] = bytes(rgb)

    def line(x0, y0, x1, y1, rgb):
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for step in range(steps + 1):
            x = round(x0 + (x1 - x0) * step / steps)
            y = round(y0 + (y1 - y0) * step / steps)
            pix(x, y, rgb)
            pix(x + 1, y, rgb)

    def panel(origin_y, items):
        vals = [v for _name, ys, _rgb in items for v in ys]
        low = min(0.0, min(vals))
        high = max(vals) if max(vals) > low else low + 1.0
        pad = 0.08 * (high - low)
        low -= pad
        high += pad

        def xy(delay, value):
            x = left + round((delay - delays_ms[0]) / (delays_ms[-1] - delays_ms[0]) * (plot_w - 1))
            y = origin_y + round((high - value) / (high - low) * (panel_h - 1))
            return x, y

        line(left, origin_y, left, origin_y + panel_h, (40, 40, 40))
        line(left, origin_y + panel_h, left + plot_w, origin_y + panel_h, (40, 40, 40))
        if low < 0 < high:
            _x0, y0 = xy(delays_ms[0], 0.0)
            _x1, y1 = xy(delays_ms[-1], 0.0)
            line(left, y0, left + plot_w, y1, (180, 180, 180))
        for _name, ys, rgb in items:
            points = [xy(d, v) for d, v in zip(delays_ms, ys)]
            for (x0, y0), (x1, y1) in zip(points, points[1:]):
                line(x0, y0, x1, y1, rgb)
            for x, y in points:
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        pix(x + dx, y + dy, rgb)
        _text(pix, left, origin_y + panel_h + 8, f"{delays_ms[0]:.0f}", (20, 20, 20))
        _text(pix, left + plot_w - 18, origin_y + panel_h + 8, f"{delays_ms[-1]:.0f}", (20, 20, 20))
        _text(pix, 8, origin_y, f"{high:.1f}", (20, 20, 20))
        _text(pix, 8, origin_y + panel_h - 8, f"{low:.1f}", (20, 20, 20))

    upper = [item for item in series if "peak" not in item[0]]
    lower = [item for item in series if "peak" in item[0]]
    panel(top, upper or series[:2])
    panel(top + panel_h + gap, lower or series[2:])
    _text(pix, left, 8, "0-50", (20, 20, 20))
    raw = zlib.compress(b"".join(b"\x00" + bytes(canvas[y * width * 3:(y + 1) * width * 3]) for y in range(height)), 9)
    png = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) + _chunk(b"IDAT", raw) + _chunk(b"IEND", b"")
    with open(path, "wb") as handle:
        handle.write(png)
