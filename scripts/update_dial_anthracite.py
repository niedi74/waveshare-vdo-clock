#!/usr/bin/env python3
"""Retarget VDO dial face background to VW T2 anthracite and remove outer rings.

Samples the anthracite tone from reference photos in docs/photos/, remaps neutral
face-background pixels, and replaces the outer white chrome ring plus the dark
and metallic bezel ring so the dial fills the full 480 px circle edge-to-edge.
Numerals, tick marks, and logos are preserved.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "vdo_dial_480_rgb565.h"
REF_PHOTOS = [
    ROOT / "docs/photos/20260524_173440.jpg",
    ROOT / "docs/photos/20260524_173456.jpg",
    ROOT / "docs/photos/20260524_173546.jpg",
    ROOT / "docs/photos/20260524_173803.jpg",
    ROOT / "docs/photos/20260524_173800.jpg",
]

CX = CY = 240


def rgb565_to_rgb(color: int) -> np.ndarray:
    r = ((color >> 11) & 0x1F) << 3
    g = ((color >> 5) & 0x3F) << 2
    b = (color & 0x1F) << 3
    return np.array([r, g, b], dtype=np.int32)


def rgb_to_rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def lum(rgb: np.ndarray) -> float:
    r, g, b = rgb
    return 0.299 * r + 0.587 * g + 0.114 * b


def sat(rgb: np.ndarray) -> int:
    return int(max(rgb) - min(rgb))


def sample_anthracite() -> np.ndarray:
    """Median anthracite from flat dial-face regions in reference photos."""
    samples: list[tuple[int, int, int]] = []
    for path in REF_PHOTOS:
        if not path.exists():
            continue
        arr = np.array(Image.open(path).convert("RGB"))
        h, w = arr.shape[:2]
        cx, cy = w // 2, h // 2
        max_r = min(cx, cy)
        for y in range(h):
            for x in range(w):
                dx, dy = x - cx, y - cy
                r = (dx * dx + dy * dy) ** 0.5
                if not (0.15 * max_r < r < 0.38 * max_r):
                    continue
                rgb = arr[y, x]
                l = lum(rgb)
                if 35 < l < 75 and sat(rgb) < 25:
                    samples.append(tuple(int(v) for v in rgb))
    if not samples:
        raise RuntimeError("No anthracite samples found in reference photos")
    return np.median(samples, axis=0).astype(np.float32)


def load_dial() -> np.ndarray:
    content = HEADER.read_text(encoding="utf-8")
    start = content.index("{") + 1
    end = content.rindex("}")
    vals = np.array(
        [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]+", content[start:end])],
        dtype=np.uint16,
    )
    if vals.size != 480 * 480:
        raise RuntimeError(f"Expected 230400 pixels, got {vals.size}")
    return vals.reshape(480, 480).copy()


def is_face_gray(rgb: np.ndarray, radius: float) -> bool:
    l = lum(rgb)
    if radius > 212 or l < 22 or l > 82 or sat(rgb) > 18:
        return False
    if l < 28 and radius > 200:
        return False
    return True


def is_white_ring(rgb: np.ndarray, radius: float) -> bool:
    return lum(rgb) > 195 and 207 <= radius <= 237


def is_marking(rgb: np.ndarray, radius: float) -> bool:
    """Keep numerals, ticks, logos, and their metallic edges."""
    l = lum(rgb)
    s = sat(rgb)
    if l >= 112:
        return True
    if s >= 20:
        return True
    if radius < 195 and l >= 58 and s >= 6:
        return True
    return False


def is_outer_bezel(radius: float, rgb: np.ndarray) -> bool:
    """Dark/metallic outer ring between face and display edge."""
    if radius < 198 or is_marking(rgb, radius):
        return False
    return True


def is_outside_circle(radius: float) -> bool:
    return radius > 239.5


def process_dial(dial: np.ndarray, target: np.ndarray) -> np.ndarray:
    old_mid = 48.0
    out = dial.copy()
    target_px = target.astype(int)
    for y in range(480):
        for x in range(480):
            rgb = rgb565_to_rgb(int(out[y, x]))
            radius = ((x - CX) ** 2 + (y - CY) ** 2) ** 0.5
            if is_white_ring(rgb, radius):
                out[y, x] = rgb_to_rgb565(*target_px)
            elif is_outside_circle(radius):
                out[y, x] = rgb_to_rgb565(*target_px)
            elif is_outer_bezel(radius, rgb):
                out[y, x] = rgb_to_rgb565(*target_px)
            elif is_face_gray(rgb, radius):
                ratio = lum(rgb) / old_mid
                new_rgb = np.clip(target * ratio, 0, 255).astype(int)
                out[y, x] = rgb_to_rgb565(*new_rgb)
    return out


def write_header(dial: np.ndarray) -> None:
    lines = [
        "#pragma once\n",
        "#include <Arduino.h>\n",
        "\n",
        "// Anthracite dial face matched to VW T2 cockpit reference; outer rings removed.\n",
        "// 480x480 RGB565 VDO dial face without hands; firmware draws live hands on top.\n",
        "static const uint16_t VDO_DIAL_480_RGB565[480 * 480] PROGMEM = {\n",
    ]
    flat = dial.flatten()
    for i in range(0, len(flat), 12):
        chunk = flat[i : i + 12]
        line = "  " + ", ".join(f"0x{v:04X}" for v in chunk)
        if i + 12 < len(flat):
            line += ","
        lines.append(line + "\n")
    lines.append("};\n")
    HEADER.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    target = sample_anthracite()
    print(f"Target anthracite RGB: {tuple(target.astype(int))}")
    dial = process_dial(load_dial(), target)
    write_header(dial)
    print(f"Updated {HEADER}")


if __name__ == "__main__":
    main()
