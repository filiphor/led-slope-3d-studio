#!/usr/bin/env python3
"""
3D LED Strip Slope / Wedge Generator (GUI + CLI)
Created by Filip Hor (Filiposhor) — https://github.com/filiphor
Licensed under Creative Commons Attribution 4.0 International (CC BY 4.0)

Generates watertight 3D-printable STL models of a tilted slope for mounting LED strips.

Features:
- Full interactive Dark-Mode CAD/Slicer GUI (launches automatically when run with no CLI flags):
    * Live Cross-Section Blueprint (X-Z) with angle, LED strip width, ground width & light throw cone
    * Live Interactive 3D Orbit Preview (convex back-face culling, drag to rotate, hover-scroll zoom)
    * Live Top-Down Slicer Build Plate Layout with Safe Boundary visualization
- Build Plate (BP) auto-fit with customizable Safe Boundary margin (default: 5 mm/edge)
- Orientation choice (Normal vs Diagonal) with automatic Normal -> Diagonal fallback
- Auto-Cutting when Diagonal doesn't fit:
    1) Multiple pieces packed side-by-side on a single bed laying Normal
    2) 1 piece per build plate laying Diagonal
- Native Windows File Explorer "Save As" dialog defaulting to Downloads
"""

import argparse
import ctypes
import math
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]
Triangle = Tuple[Vec3, Vec3, Vec3]


# =====================================================================
# 1. CORE 3D GEOMETRY & BUILD PLATE LAYOUT ENGINE
# =====================================================================

def compute_normal(v0: Vec3, v1: Vec3, v2: Vec3) -> Vec3:
    """Compute unit normal vector for a triangle (v0, v1, v2) using right-hand rule."""
    ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def transform_triangles(
    triangles: List[Triangle],
    rot_z_rad: float = 0.0,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
) -> List[Triangle]:
    """Rotate triangles around the Z-axis by rot_z_rad, then translate by (tx, ty, tz)."""
    cos_a = math.cos(rot_z_rad)
    sin_a = math.sin(rot_z_rad)

    def xform(pt: Vec3) -> Vec3:
        x, y, z = pt
        rx = x * cos_a - y * sin_a + tx
        ry = x * sin_a + y * cos_a + ty
        rz = z + tz
        return (rx, ry, rz)

    return [(xform(v0), xform(v1), xform(v2)) for v0, v1, v2 in triangles]


def build_led_slope_mesh(
    strip_width: float,
    angle_deg: float,
    extrude_length: float = 100.0,
    min_base_height: float = 0.0,
    center_xy: bool = True,
) -> Tuple[List[Triangle], dict]:
    """
    Build the triangles for a single LED strip slope prism.

    Cross-section in the X-Z plane (extruded along Y):
      - Slope Left:   (x_left,  min_base_height)
      - Slope Right:  (x_right, min_base_height + W * sin(angle))
      - Ground Right: (x_right, 0)
      - Ground Left:  (x_left,  0)
    """
    if strip_width <= 0:
        raise ValueError("Strip width must be greater than 0.")
    if not (0 < angle_deg < 90):
        raise ValueError("Angle must be between 0 and 90 degrees (exclusive).")
    if extrude_length <= 0:
        raise ValueError("Extrusion length must be greater than 0.")
    if min_base_height < 0:
        raise ValueError("Minimum base height cannot be negative.")

    rad = math.radians(angle_deg)
    base_width = strip_width * math.cos(rad)
    z_slope_left = min_base_height
    z_slope_right = min_base_height + strip_width * math.sin(rad)

    if center_xy:
        x_left = -base_width / 2.0
        x_right = base_width / 2.0
        y0 = -extrude_length / 2.0
        y1 = extrude_length / 2.0
    else:
        x_left = 0.0
        x_right = base_width
        y0 = 0.0
        y1 = extrude_length

    metrics = {
        "strip_width_mm": strip_width,
        "angle_deg": angle_deg,
        "extrude_length_mm": extrude_length,
        "ground_base_width_mm": base_width,
        "back_wall_height_mm": z_slope_right,
        "front_edge_height_mm": z_slope_left,
    }

    triangles: List[Triangle] = []

    def add_quad(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> None:
        """Add two triangles for quad (p0, p1, p2, p3) in counter-clockwise order."""
        triangles.append((p0, p1, p2))
        triangles.append((p0, p2, p3))

    if min_base_height == 0.0:
        f_left: Vec3 = (x_left, y0, 0.0)
        f_right_bottom: Vec3 = (x_right, y0, 0.0)
        f_right_top: Vec3 = (x_right, y0, z_slope_right)

        b_left: Vec3 = (x_left, y1, 0.0)
        b_right_bottom: Vec3 = (x_right, y1, 0.0)
        b_right_top: Vec3 = (x_right, y1, z_slope_right)

        # 1. Front cap (Y = y0, normal -Y)
        triangles.append((f_left, f_right_bottom, f_right_top))
        # 2. Back cap (Y = y1, normal +Y)
        triangles.append((b_left, b_right_top, b_right_bottom))
        # 3. Bottom ground face (Z = 0, normal -Z)
        add_quad(f_left, b_left, b_right_bottom, f_right_bottom)
        # 4. Vertical back wall (X = x_right, normal +X)
        add_quad(f_right_bottom, b_right_bottom, b_right_top, f_right_top)
        # 5. Sloped LED strip wall (normal pointing up-left, last 2 triangles)
        add_quad(f_left, f_right_top, b_right_top, b_left)
    else:
        f_g_left: Vec3 = (x_left, y0, 0.0)
        f_g_right: Vec3 = (x_right, y0, 0.0)
        f_s_right: Vec3 = (x_right, y0, z_slope_right)
        f_s_left: Vec3 = (x_left, y0, z_slope_left)

        b_g_left: Vec3 = (x_left, y1, 0.0)
        b_g_right: Vec3 = (x_right, y1, 0.0)
        b_s_right: Vec3 = (x_right, y1, z_slope_right)
        b_s_left: Vec3 = (x_left, y1, z_slope_left)

        add_quad(f_g_left, f_g_right, f_s_right, f_s_left)
        add_quad(b_g_left, b_s_left, b_s_right, b_g_right)
        add_quad(f_g_left, b_g_left, b_g_right, f_g_right)
        add_quad(f_g_right, b_g_right, b_s_right, f_s_right)
        add_quad(f_g_left, f_s_left, b_s_left, b_g_left)
        # Sloped LED strip wall (last 2 triangles)
        add_quad(f_s_left, f_s_right, b_s_right, b_s_left)

    return triangles, metrics


def parse_bp_size(bp_str: str) -> Tuple[float, float]:
    """
    Parse build plate size string into (bp_x, bp_y) in mm.
    Supports formats like: '256', '256x256', '250x210', '220,220', '220 220'.
    """
    cleaned = bp_str.lower().replace("mm", "").strip()
    for sep in ("x", ",", "*", " "):
        if sep in cleaned:
            parts = [p.strip() for p in cleaned.split(sep) if p.strip()]
            if len(parts) == 2:
                bx, by = float(parts[0]), float(parts[1])
                if bx <= 0 or by <= 0:
                    raise ValueError("Build plate dimensions must be > 0.")
                return (bx, by)
    val = float(cleaned)
    if val <= 0:
        raise ValueError("Build plate dimension must be > 0.")
    return (val, val)


def _optimal_diagonal_fit(usable_x: float, usable_y: float, base_width: float) -> Tuple[float, float]:
    """
    Find the exact optimal rotation angle `rot_z_rad` (in [0, pi/2]) that maximizes
    strip length `L` of width `base_width` inside `usable_x` x `usable_y`.
    """
    def feasible_len(phi: float) -> float:
        s = math.sin(phi)
        c = math.cos(phi)
        rem_x = usable_x - base_width * c
        rem_y = usable_y - base_width * s
        if rem_x <= 0 or rem_y <= 0:
            return 0.0
        lx = rem_x / s if s > 1e-9 else float("inf")
        ly = rem_y / c if c > 1e-9 else float("inf")
        return max(0.0, min(lx, ly))

    lo, hi = 1e-5, math.pi / 2.0 - 1e-5
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        s, c = math.sin(mid), math.cos(mid)
        lx = (usable_x - base_width * c) / s
        ly = (usable_y - base_width * s) / c
        if lx > ly:
            lo = mid
        else:
            hi = mid

    best_phi = 0.5 * (lo + hi)
    best_len = feasible_len(best_phi)

    bed_phi = math.atan2(usable_x, usable_y)
    bed_len = feasible_len(bed_phi)
    if bed_len > best_len:
        best_len, best_phi = bed_len, bed_phi

    return best_len, best_phi


def compute_bp_limits(bp_x: float, bp_y: float, base_width: float, margin_mm: float = 5.0) -> dict:
    """
    Compute maximum printable strip length on a (bp_x, bp_y) build plate
    with a `margin_mm` safe boundary from every edge.
    """
    if margin_mm < 0:
        raise ValueError("Safe boundary margin cannot be negative.")

    usable_x = bp_x - 2.0 * margin_mm
    usable_y = bp_y - 2.0 * margin_mm
    if usable_x <= 0 or usable_y <= 0:
        raise ValueError(
            f"Safe boundary ({margin_mm:g} mm/edge) leaves no room on {bp_x:g}x{bp_y:g} mm bed."
        )

    bed_long = max(usable_x, usable_y)
    bed_short = min(usable_x, usable_y)
    normal_rot_z_rad = (math.pi / 2.0) if usable_x > usable_y else 0.0

    if base_width > bed_short:
        max_normal = 0.0
    else:
        max_normal = bed_long

    max_diag, rot_z_rad = _optimal_diagonal_fit(usable_x, usable_y, base_width)

    return {
        "bp_x": bp_x,
        "bp_y": bp_y,
        "margin_mm": margin_mm,
        "usable_x": usable_x,
        "usable_y": usable_y,
        "bed_long": bed_long,
        "bed_short": bed_short,
        "max_normal_mm": max_normal,
        "max_diagonal_mm": max_diag,
        "normal_rot_z_rad": normal_rot_z_rad,
        "diag_rot_z_rad": rot_z_rad,
        "diag_rot_z_deg": math.degrees(rot_z_rad),
    }


def split_length_into_segments(
    total_length: float, max_piece_length: float, equal_segments: bool = False
) -> List[float]:
    """Split `total_length` into segments each <= `max_piece_length`."""
    if max_piece_length <= 0:
        raise ValueError("Strip ground footprint is wider than the usable build plate.")

    if total_length <= max_piece_length + 1e-6:
        return [total_length]

    num_pieces = math.ceil(total_length / max_piece_length - 1e-9)
    if equal_segments:
        piece_len = total_length / num_pieces
        return [piece_len] * num_pieces

    segments: List[float] = []
    remaining = total_length
    while remaining > max_piece_length + 1e-6:
        segments.append(max_piece_length)
        remaining -= max_piece_length
    if remaining > 1e-6:
        segments.append(remaining)
    return segments


def _rect_footprint_xy(
    base_width: float, length: float, rot_z_rad: float, tx: float, ty: float
) -> List[Tuple[float, float]]:
    """Return the 4 XY corners of a placed piece."""
    hw = base_width / 2.0
    hl = length / 2.0
    local = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    c, s = math.cos(rot_z_rad), math.sin(rot_z_rad)
    return [(x * c - y * s + tx, x * s + y * c + ty) for x, y in local]


def plan_build_plate_layout(
    strip_width: float,
    angle_deg: float,
    total_length: float,
    min_base_height: float,
    bp_x: float,
    bp_y: float,
    margin_mm: float = 5.0,
    pref_mode: str = "normal",
    cut_mode: str = "normal_multi",
    equal_cuts: bool = False,
    part_gap_mm: float = 4.0,
) -> dict:
    """
    Compute the complete auto-fit / auto-cut build plate plan.
    Used identically by both the GUI and CLI.
    """
    if strip_width <= 0:
        raise ValueError("Strip width must be > 0 mm.")
    if not (0 < angle_deg < 90):
        raise ValueError("Angle must be between 0 and 90 degrees.")
    if total_length <= 0:
        raise ValueError("Total length must be > 0 mm.")
    if min_base_height < 0:
        raise ValueError("Min base height cannot be negative.")
    if part_gap_mm < 0:
        raise ValueError("Part gap cannot be negative.")

    rad = math.radians(angle_deg)
    base_width = strip_width * math.cos(rad)
    back_height = min_base_height + strip_width * math.sin(rad)
    bp_info = compute_bp_limits(bp_x, bp_y, base_width, margin_mm=margin_mm)

    max_normal = bp_info["max_normal_mm"]
    max_diag = bp_info["max_diagonal_mm"]
    norm_rot = bp_info["normal_rot_z_rad"]
    diag_rot = bp_info["diag_rot_z_rad"]

    if max_normal <= 0 and max_diag <= 0:
        raise ValueError(
            f"Ground base width ({base_width:.1f} mm) is too wide for usable bed "
            f"({bp_info['usable_x']:.1f}x{bp_info['usable_y']:.1f} mm)."
        )

    needs_cutting = False
    if pref_mode == "normal":
        if max_normal > 0 and total_length <= max_normal + 1e-6:
            final_layout = "single_normal"
            status_level = "ok"
            status_text = f"Fits NORMAL as 1 piece ({total_length:.1f} mm <= {max_normal:.1f} mm)"
        elif total_length <= max_diag + 1e-6:
            final_layout = "single_diagonal"
            status_level = "info"
            status_text = (
                f"Exceeds Normal ({max_normal:.1f} mm) -> Auto-fitted DIAGONAL as 1 piece "
                f"({total_length:.1f} mm <= {max_diag:.1f} mm)!"
            )
        else:
            needs_cutting = True
            final_layout = "cut_diagonal_single" if cut_mode == "diagonal_single" else "cut_normal_multi"
            status_level = "warn"
            status_text = (
                f"Exceeds Normal ({max_normal:.1f} mm) & Diagonal ({max_diag:.1f} mm) -> Auto-Cut Active!"
            )
    else:  # pref_mode == "diagonal"
        if total_length <= max_diag + 1e-6:
            final_layout = "single_diagonal"
            status_level = "ok"
            status_text = f"Fits DIAGONAL as 1 piece ({total_length:.1f} mm <= {max_diag:.1f} mm)"
        else:
            needs_cutting = True
            final_layout = "cut_diagonal_single" if cut_mode == "diagonal_single" else "cut_normal_multi"
            status_level = "warn"
            status_text = f"Exceeds Diagonal ({max_diag:.1f} mm) -> Auto-Cut Active!"

    norm_preview_segs = (
        split_length_into_segments(total_length, max_normal, equal_cuts) if max_normal > 0 else []
    )
    diag_preview_segs = (
        split_length_into_segments(total_length, max_diag, equal_cuts) if max_diag > 0 else []
    )

    plates: List[dict] = []

    if final_layout == "single_normal":
        plates.append(
            {
                "plate_index": 1,
                "label": f"1 Piece — Normal ({total_length:.1f} mm)",
                "file_suffix": "",
                "pieces": [
                    {
                        "length_mm": total_length,
                        "base_width_mm": base_width,
                        "rot_z_rad": norm_rot,
                        "tx": 0.0,
                        "ty": 0.0,
                        "corners_xy": _rect_footprint_xy(base_width, total_length, norm_rot, 0.0, 0.0),
                    }
                ],
            }
        )

    elif final_layout == "single_diagonal":
        plates.append(
            {
                "plate_index": 1,
                "label": f"1 Piece — Diagonal {bp_info['diag_rot_z_deg']:.1f}° ({total_length:.1f} mm)",
                "file_suffix": "",
                "pieces": [
                    {
                        "length_mm": total_length,
                        "base_width_mm": base_width,
                        "rot_z_rad": diag_rot,
                        "tx": 0.0,
                        "ty": 0.0,
                        "corners_xy": _rect_footprint_xy(base_width, total_length, diag_rot, 0.0, 0.0),
                    }
                ],
            }
        )

    elif final_layout == "cut_normal_multi":
        if max_normal <= 0:
            raise ValueError("Piece ground width exceeds usable bed width for Normal printing; choose Diagonal.")
        segments = norm_preview_segs
        max_per_bed = max(1, int((bp_info["bed_short"] + part_gap_mm) // (base_width + part_gap_mm)))
        num_beds = math.ceil(len(segments) / max_per_bed)

        for b_idx in range(num_beds):
            batch = segments[b_idx * max_per_bed : (b_idx + 1) * max_per_bed]
            count = len(batch)
            span = count * base_width + (count - 1) * part_gap_mm
            start_offset = -span / 2.0 + base_width / 2.0

            pieces = []
            for idx, seg_len in enumerate(batch):
                shift = start_offset + idx * (base_width + part_gap_mm)
                if abs(norm_rot) < 1e-6:
                    tx, ty = shift, 0.0
                else:
                    tx, ty = 0.0, shift
                pieces.append(
                    {
                        "length_mm": seg_len,
                        "base_width_mm": base_width,
                        "rot_z_rad": norm_rot,
                        "tx": tx,
                        "ty": ty,
                        "corners_xy": _rect_footprint_xy(base_width, seg_len, norm_rot, tx, ty),
                    }
                )

            seg_str = " + ".join(f"{s:.1f}" for s in batch)
            plates.append(
                {
                    "plate_index": b_idx + 1,
                    "label": f"Bed {b_idx + 1}/{num_beds} — {count} pcs Normal ({seg_str} mm)",
                    "file_suffix": "" if num_beds == 1 else f"_bed{b_idx + 1}",
                    "pieces": pieces,
                }
            )

    elif final_layout == "cut_diagonal_single":
        segments = diag_preview_segs
        num_plates = len(segments)
        for p_idx, seg_len in enumerate(segments):
            plates.append(
                {
                    "plate_index": p_idx + 1,
                    "label": f"Plate {p_idx + 1}/{num_plates} — 1 pc Diagonal ({seg_len:.1f} mm)",
                    "file_suffix": "" if num_plates == 1 else f"_plate{p_idx + 1}_{seg_len:.1f}mm",
                    "pieces": [
                        {
                            "length_mm": seg_len,
                            "base_width_mm": base_width,
                            "rot_z_rad": diag_rot,
                            "tx": 0.0,
                            "ty": 0.0,
                            "corners_xy": _rect_footprint_xy(base_width, seg_len, diag_rot, 0.0, 0.0),
                        }
                    ],
                }
            )

    return {
        "strip_width_mm": strip_width,
        "angle_deg": angle_deg,
        "total_length_mm": total_length,
        "min_base_height_mm": min_base_height,
        "ground_base_width_mm": base_width,
        "back_wall_height_mm": back_height,
        "bp_info": bp_info,
        "needs_cutting": needs_cutting,
        "final_layout": final_layout,
        "status_level": status_level,
        "status_text": status_text,
        "norm_preview_segs": norm_preview_segs,
        "diag_preview_segs": diag_preview_segs,
        "plates": plates,
    }


def build_plate_mesh(plan: dict, plate_index_zero_based: int = 0) -> List[Triangle]:
    """Build the combined 3D STL triangle list for a specific plate in `plan`."""
    plate = plan["plates"][plate_index_zero_based]
    combined: List[Triangle] = []
    for pc in plate["pieces"]:
        tris, _ = build_led_slope_mesh(
            strip_width=plan["strip_width_mm"],
            angle_deg=plan["angle_deg"],
            extrude_length=pc["length_mm"],
            min_base_height=plan["min_base_height_mm"],
            center_xy=True,
        )
        placed = transform_triangles(tris, rot_z_rad=pc["rot_z_rad"], tx=pc["tx"], ty=pc["ty"], tz=0.0)
        combined.extend(placed)
    return combined


def build_piece_convex_faces(
    strip_width: float,
    angle_deg: float,
    extrude_length: float,
    min_base_height: float = 0.0,
    rot_z_rad: float = 0.0,
    tx: float = 0.0,
    ty: float = 0.0,
) -> List[Tuple[List[Vec3], Vec3, str]]:
    """
    Build exact untriangulated convex polygonal faces `(verts, outward_normal, role)`
    for a single placed LED slope piece.

    Because each piece is a strictly convex prism, back-face culling (`normal · view_to_cam > 0`)
    eliminates 100% of hidden surfaces, and front-facing polygons never overlap or clip through each other.
    """
    rad = math.radians(angle_deg)
    base_width = strip_width * math.cos(rad)
    x_left = -base_width / 2.0
    x_right = base_width / 2.0
    y0 = -extrude_length / 2.0
    y1 = extrude_length / 2.0
    z_slope_left = min_base_height
    z_slope_right = min_base_height + strip_width * math.sin(rad)

    cos_a = math.cos(rot_z_rad)
    sin_a = math.sin(rot_z_rad)

    def xform(pt: Vec3) -> Vec3:
        x, y, z = pt
        return (
            x * cos_a - y * sin_a + tx,
            x * sin_a + y * cos_a + ty,
            z,
        )

    raw_faces: List[Tuple[List[Vec3], str]] = []
    if min_base_height == 0.0:
        f_left: Vec3 = (x_left, y0, 0.0)
        f_rb: Vec3 = (x_right, y0, 0.0)
        f_rt: Vec3 = (x_right, y0, z_slope_right)

        b_left: Vec3 = (x_left, y1, 0.0)
        b_rb: Vec3 = (x_right, y1, 0.0)
        b_rt: Vec3 = (x_right, y1, z_slope_right)

        raw_faces = [
            ([f_left, f_rb, f_rt], "cap"),
            ([b_left, b_rt, b_rb], "cap"),
            ([f_left, b_left, b_rb, f_rb], "ground"),
            ([f_rb, b_rb, b_rt, f_rt], "wall"),
            ([f_left, f_rt, b_rt, b_left], "led_slope"),
        ]
    else:
        f_gl: Vec3 = (x_left, y0, 0.0)
        f_gr: Vec3 = (x_right, y0, 0.0)
        f_sr: Vec3 = (x_right, y0, z_slope_right)
        f_sl: Vec3 = (x_left, y0, z_slope_left)

        b_gl: Vec3 = (x_left, y1, 0.0)
        b_gr: Vec3 = (x_right, y1, 0.0)
        b_sr: Vec3 = (x_right, y1, z_slope_right)
        b_sl: Vec3 = (x_left, y1, z_slope_left)

        raw_faces = [
            ([f_gl, f_gr, f_sr, f_sl], "cap"),
            ([b_gl, b_sl, b_sr, b_gr], "cap"),
            ([f_gl, b_gl, b_gr, f_gr], "ground"),
            ([f_gr, b_gr, b_sr, f_sr], "wall"),
            ([f_gl, f_sl, b_sl, b_gl], "wall"),
            ([f_sl, f_sr, b_sr, b_sl], "led_slope"),
        ]

    out: List[Tuple[List[Vec3], Vec3, str]] = []
    for verts, role in raw_faces:
        wv = [xform(p) for p in verts]
        n = compute_normal(wv[0], wv[1], wv[2])
        out.append((wv, n, role))
    return out


def write_binary_stl(
    filepath: Path,
    triangles: List[Triangle],
    header_text: str = "LED Slope Studio by Filip Hor (Filiposhor) - CC BY 4.0",
) -> None:
    """Write triangles to a standard binary STL file."""
    header = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b"\x00")
    with filepath.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for v0, v1, v2 in triangles:
            nx, ny, nz = compute_normal(v0, v1, v2)
            f.write(
                struct.pack(
                    "<12fH",
                    nx, ny, nz,
                    v0[0], v0[1], v0[2],
                    v1[0], v1[1], v1[2],
                    v2[0], v2[1], v2[2],
                    0,
                )
            )


def export_plan_to_stls(plan: dict, base_output_path: Path) -> List[Tuple[Path, str]]:
    """Write all plates in `plan` to disk and return list of (saved_path, label)."""
    base_output_path.parent.mkdir(parents=True, exist_ok=True)
    stem = base_output_path.stem
    suffix = base_output_path.suffix or ".stl"
    parent_dir = base_output_path.parent

    saved: List[Tuple[Path, str]] = []
    for idx, plate in enumerate(plan["plates"]):
        tris = build_plate_mesh(plan, idx)
        file_sfx = plate["file_suffix"]
        out_path = parent_dir / f"{stem}{file_sfx}{suffix}" if file_sfx else base_output_path
        write_binary_stl(out_path, tris)
        saved.append((out_path, plate["label"]))
    return saved


def choose_save_path(default_dir: Path, default_filename: str, parent=None) -> Optional[Path]:
    """Open a native file manager 'Save As' dialog defaulting to default_dir."""
    try:
        import tkinter as tk
        from tkinter import filedialog

        own_root = None
        if parent is None:
            own_root = tk.Tk()
            own_root.withdraw()
            own_root.attributes("-topmost", True)
            dialog_parent = own_root
        else:
            dialog_parent = parent

        selected = filedialog.asksaveasfilename(
            parent=dialog_parent,
            title="Save LED Slope 3D Model (.stl)",
            initialdir=str(default_dir),
            initialfile=default_filename,
            defaultextension=".stl",
            filetypes=[("STL 3D Model", "*.stl"), ("All Files", "*.*")],
        )
        if own_root is not None:
            own_root.destroy()
        if not selected:
            return None
        return Path(selected)
    except Exception as exc:
        print(f"Could not open file manager dialog ({exc}), falling back to Downloads.")
        return default_dir / default_filename


# =====================================================================
# 2. FULL INTERACTIVE DESKTOP GUI (DARK SLICER / CAD STUDIO)
# =====================================================================

BP_PRESETS: List[Tuple[str, str, str]] = [
    ("Bambu X1 / P1 / A1 (256×256)", "256", "256"),
    ("Bambu A1 Mini (180×180)", "180", "180"),
    ("Creality Ender 3 / K1 (220×220)", "220", "220"),
    ("Prusa MK3 / MK4 / Core One (250×210)", "250", "210"),
    ("Voron / K1 Max / Large (300×300)", "300", "300"),
    ("Voron 350 / XL (350×350)", "350", "350"),
    ("Custom Dimensions", "", ""),
]


def _enable_windows_dpi_awareness() -> None:
    if sys.platform == "win32":
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:
                pass


def launch_gui() -> None:
    """Launch the interactive LED Slope Studio GUI."""
    _enable_windows_dpi_awareness()
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    BG_ROOT = "#13161c"
    BG_PANEL = "#1b1f28"
    BG_CARD = "#232834"
    BG_INPUT = "#151821"
    FG_TEXT = "#eef2f6"
    FG_MUTED = "#94a3b8"
    ACCENT = "#00d4aa"
    ACCENT_HOVER = "#00f0c0"
    ACCENT_BLUE = "#38bdf8"
    ACCENT_AMBER = "#f59e0b"
    ACCENT_RED = "#ef4444"
    BORDER = "#2e3545"

    root = tk.Tk()
    root.title("LED Strip 3D Slope Studio — by Filip Hor (Filiposhor)")
    root.configure(bg=BG_ROOT)

    sw = root.winfo_screenwidth()
    sh = root.winfo_screenheight()
    win_w = min(1360, int(sw * 0.90))
    win_h = min(840, int(sh * 0.86))
    pos_x = max(10, (sw - win_w) // 2)
    pos_y = max(10, (sh - win_h) // 2)
    root.geometry(f"{win_w}x{win_h}+{pos_x}+{pos_y}")
    root.minsize(980, 620)

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass

    style.configure("TFrame", background=BG_PANEL)
    style.configure("Card.TFrame", background=BG_CARD)
    style.configure("TLabel", background=BG_CARD, foreground=FG_TEXT, font=("Segoe UI", 9))
    style.configure("Muted.TLabel", background=BG_CARD, foreground=FG_MUTED, font=("Segoe UI", 8))
    style.configure("CardTitle.TLabel", background=BG_CARD, foreground=ACCENT, font=("Segoe UI Semibold", 10))
    style.configure(
        "TRadiobutton",
        background=BG_CARD,
        foreground=FG_TEXT,
        font=("Segoe UI", 9),
        indicatorcolor=BG_INPUT,
        focuscolor=BG_CARD,
    )
    style.map(
        "TRadiobutton",
        background=[("active", BG_CARD)],
        indicatorcolor=[("selected", ACCENT), ("!selected", BG_INPUT)],
    )
    style.configure(
        "TCheckbutton",
        background=BG_CARD,
        foreground=FG_TEXT,
        font=("Segoe UI", 9),
        indicatorcolor=BG_INPUT,
        focuscolor=BG_CARD,
    )
    style.map(
        "TCheckbutton",
        background=[("active", BG_CARD)],
        indicatorcolor=[("selected", ACCENT), ("!selected", BG_INPUT)],
    )
    style.configure(
        "TCombobox",
        fieldbackground=BG_INPUT,
        background=BG_CARD,
        foreground=FG_TEXT,
        arrowcolor=FG_TEXT,
    )
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", BG_INPUT)],
        selectbackground=[("readonly", BG_INPUT)],
        selectforeground=[("readonly", FG_TEXT)],
    )
    style.configure(
        "Horizontal.TScale",
        background=BG_CARD,
        troughcolor=BG_INPUT,
        bordercolor=BG_CARD,
        lightcolor=ACCENT,
        darkcolor=ACCENT,
    )

    var_width = tk.StringVar(value="10.0")
    var_angle = tk.StringVar(value="30.0")
    var_length = tk.StringVar(value="300.0")
    var_base = tk.StringVar(value="0.0")

    var_preset = tk.StringVar(value=BP_PRESETS[0][0])
    var_bp_x = tk.StringVar(value="256")
    var_bp_y = tk.StringVar(value="256")
    var_margin = tk.StringVar(value="5.0")

    var_pref_mode = tk.StringVar(value="normal")
    var_cut_mode = tk.StringVar(value="normal_multi")
    var_equal_cuts = tk.BooleanVar(value=False)
    var_part_gap = tk.StringVar(value="4.0")
    var_ask_on_cut = tk.BooleanVar(value=True)

    downloads_default = Path.home() / "Downloads"
    var_save_dir = tk.StringVar(value=str(downloads_default))

    state: Dict[str, object] = {
        "ready": False,
        "plan": None,
        "error": None,
        "active_plate_idx": 0,
        "updating_slider": False,
        "cam_yaw": -0.65,
        "cam_pitch": 0.55,
        "cam_zoom": 1.35,
        "cam_pan_x": 0.0,
        "cam_pan_y": 0.0,
        "cam_target": (0.0, 0.0, 0.0),
        "drag_last": None,
        "drag_mode": None,
    }

    outer = tk.Frame(root, bg=BG_ROOT)
    outer.pack(fill="both", expand=True, padx=8, pady=8)
    outer.columnconfigure(0, weight=0, minsize=370)
    outer.columnconfigure(1, weight=1)
    outer.rowconfigure(0, weight=1)

    sidebar_wrap = tk.Frame(outer, bg=BG_PANEL, width=380, highlightbackground=BORDER, highlightthickness=1)
    sidebar_wrap.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
    sidebar_wrap.grid_propagate(False)
    sidebar_wrap.rowconfigure(1, weight=1)
    sidebar_wrap.columnconfigure(0, weight=1)

    hdr = tk.Frame(sidebar_wrap, bg=BG_PANEL, padx=14, pady=10)
    hdr.grid(row=0, column=0, sticky="ew")
    tk.Label(
        hdr, text="LED SLOPE 3D STUDIO", bg=BG_PANEL, fg=FG_TEXT, font=("Segoe UI Bold", 12)
    ).pack(anchor="w")
    tk.Label(
        hdr,
        text="by Filip Hor (Filiposhor) • CC BY 4.0",
        bg=BG_PANEL,
        fg=ACCENT,
        font=("Segoe UI Semibold", 8),
    ).pack(anchor="w")

    ctrl_canvas = tk.Canvas(sidebar_wrap, bg=BG_PANEL, highlightthickness=0, bd=0)
    ctrl_scroll = ttk.Scrollbar(sidebar_wrap, orient="vertical", command=ctrl_canvas.yview)
    ctrl_inner = tk.Frame(ctrl_canvas, bg=BG_PANEL, padx=10, pady=2)

    ctrl_window_id = ctrl_canvas.create_window((0, 0), window=ctrl_inner, anchor="nw")
    ctrl_canvas.configure(yscrollcommand=ctrl_scroll.set)

    ctrl_canvas.grid(row=1, column=0, sticky="nsew")
    ctrl_scroll.grid(row=1, column=1, sticky="ns")

    def _on_ctrl_inner_config(_event=None):
        ctrl_canvas.configure(scrollregion=ctrl_canvas.bbox("all"))

    def _on_ctrl_canvas_config(event):
        ctrl_canvas.itemconfigure(ctrl_window_id, width=event.width)

    ctrl_inner.bind("<Configure>", _on_ctrl_inner_config)
    ctrl_canvas.bind("<Configure>", _on_ctrl_canvas_config)

    def _on_sidebar_wheel(event):
        ctrl_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    ctrl_canvas.bind("<Enter>", lambda _e: ctrl_canvas.bind_all("<MouseWheel>", _on_sidebar_wheel))
    ctrl_canvas.bind("<Leave>", lambda _e: ctrl_canvas.unbind_all("<MouseWheel>"))

    def make_card(title: str) -> tk.Frame:
        card = tk.Frame(ctrl_inner, bg=BG_CARD, padx=10, pady=8, highlightbackground=BORDER, highlightthickness=1)
        card.pack(fill="x", pady=(0, 8))
        ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor="w", pady=(0, 6))
        return card

    def make_slider_row(
        parent: tk.Frame,
        label_text: str,
        var: tk.StringVar,
        min_v: float,
        max_v: float,
        fmt: str = "{:.1f}",
    ) -> ttk.Scale:
        row = tk.Frame(parent, bg=BG_CARD)
        row.pack(fill="x", pady=3)

        top = tk.Frame(row, bg=BG_CARD)
        top.pack(fill="x")
        ttk.Label(top, text=label_text).pack(side="left")

        ent = tk.Entry(
            top,
            textvariable=var,
            width=8,
            justify="right",
            bg=BG_INPUT,
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            relief="flat",
            font=("Consolas", 9),
            highlightbackground=BORDER,
            highlightcolor=ACCENT,
            highlightthickness=1,
        )
        ent.pack(side="right")

        def on_scale_move(val_str: str):
            if not state["ready"] or state["updating_slider"]:
                return
            state["updating_slider"] = True
            try:
                var.set(fmt.format(float(val_str)))
            finally:
                state["updating_slider"] = False
            refresh_all()

        scale = ttk.Scale(row, from_=min_v, to=max_v, orient="horizontal", command=on_scale_move)
        try:
            scale.set(float(var.get()))
        except ValueError:
            pass
        scale.pack(fill="x", pady=(2, 0))

        def on_var_edit(*_args):
            if not state["ready"] or state["updating_slider"]:
                return
            try:
                v = float(var.get())
                state["updating_slider"] = True
                scale.set(max(min_v, min(max_v, v)))
                state["updating_slider"] = False
            except ValueError:
                state["updating_slider"] = False
            refresh_all()

        var.trace_add("write", on_var_edit)
        return scale

    card_geom = make_card("1. LED Strip & Slope Geometry")
    make_slider_row(card_geom, "LED Strip Width (mm)", var_width, 3.0, 40.0, "{:.1f}")
    make_slider_row(card_geom, "Tilt Angle (degrees)", var_angle, 5.0, 85.0, "{:.1f}")
    make_slider_row(card_geom, "Total Strip Length (mm)", var_length, 20.0, 1500.0, "{:.0f}")
    make_slider_row(card_geom, "Min Tip Thickness (mm)", var_base, 0.0, 4.0, "{:.2f}")

    lbl_geom_stats = tk.Label(
        card_geom,
        text="",
        bg="#191d26",
        fg=ACCENT_BLUE,
        font=("Consolas", 8),
        justify="left",
        anchor="w",
        padx=8,
        pady=5,
    )
    lbl_geom_stats.pack(fill="x", pady=(6, 0))

    card_bp = make_card("2. Build Plate & Safe Boundary")
    ttk.Label(card_bp, text="Printer / Build Plate Preset:", style="Muted.TLabel").pack(anchor="w")
    cb_preset = ttk.Combobox(
        card_bp,
        textvariable=var_preset,
        values=[p[0] for p in BP_PRESETS],
        state="readonly",
        font=("Segoe UI", 9),
    )
    cb_preset.pack(fill="x", pady=(2, 6))

    bp_xy_row = tk.Frame(card_bp, bg=BG_CARD)
    bp_xy_row.pack(fill="x", pady=2)
    ttk.Label(bp_xy_row, text="Bed Size X × Y (mm):").pack(side="left")

    ent_bpx = tk.Entry(
        bp_xy_row,
        textvariable=var_bp_x,
        width=6,
        justify="center",
        bg=BG_INPUT,
        fg=FG_TEXT,
        insertbackground=FG_TEXT,
        relief="flat",
        font=("Consolas", 9),
        highlightbackground=BORDER,
        highlightcolor=ACCENT,
        highlightthickness=1,
    )
    ent_bpy = tk.Entry(
        bp_xy_row,
        textvariable=var_bp_y,
        width=6,
        justify="center",
        bg=BG_INPUT,
        fg=FG_TEXT,
        insertbackground=FG_TEXT,
        relief="flat",
        font=("Consolas", 9),
        highlightbackground=BORDER,
        highlightcolor=ACCENT,
        highlightthickness=1,
    )
    ent_bpy.pack(side="right")
    tk.Label(bp_xy_row, text="×", bg=BG_CARD, fg=FG_MUTED, font=("Segoe UI", 9)).pack(side="right", padx=4)
    ent_bpx.pack(side="right")

    def on_preset_selected(_event=None):
        sel = var_preset.get()
        for name, bx, by in BP_PRESETS:
            if name == sel and bx and by:
                var_bp_x.set(bx)
                var_bp_y.set(by)
                break
        refresh_all()

    cb_preset.bind("<<ComboboxSelected>>", on_preset_selected)
    var_bp_x.trace_add("write", lambda *_: refresh_all())
    var_bp_y.trace_add("write", lambda *_: refresh_all())

    make_slider_row(card_bp, "Safe Boundary Margin (mm/edge)", var_margin, 0.0, 25.0, "{:.1f}")

    lbl_bp_limits = tk.Label(
        card_bp,
        text="",
        bg="#191d26",
        fg=FG_MUTED,
        font=("Consolas", 8),
        justify="left",
        anchor="w",
        padx=8,
        pady=5,
    )
    lbl_bp_limits.pack(fill="x", pady=(6, 0))

    card_orient = make_card("3. Orientation & Auto-Cut Mode")
    ttk.Label(card_orient, text="Preferred Print Orientation:", style="Muted.TLabel").pack(anchor="w")

    orient_row = tk.Frame(card_orient, bg=BG_CARD)
    orient_row.pack(fill="x", pady=(2, 6))
    ttk.Radiobutton(
        orient_row, text="Normal (0°)", variable=var_pref_mode, value="normal", command=lambda: refresh_all()
    ).pack(side="left", padx=(0, 14))
    ttk.Radiobutton(
        orient_row, text="Diagonal", variable=var_pref_mode, value="diagonal", command=lambda: refresh_all()
    ).pack(side="left")

    lbl_status_badge = tk.Label(
        card_orient,
        text="",
        bg="#16322b",
        fg=ACCENT,
        font=("Segoe UI Semibold", 8),
        wraplength=310,
        justify="left",
        anchor="w",
        padx=8,
        pady=6,
    )
    lbl_status_badge.pack(fill="x", pady=(2, 6))

    cut_box = tk.Frame(card_orient, bg="#1b202b", padx=8, pady=6, highlightbackground=BORDER, highlightthickness=1)
    cut_box.pack(fill="x", pady=(2, 0))
    lbl_cut_header = tk.Label(
        cut_box,
        text="When Cutting Is Needed:",
        bg="#1b202b",
        fg=FG_MUTED,
        font=("Segoe UI Semibold", 8),
    )
    lbl_cut_header.pack(anchor="w", pady=(0, 4))

    rb_cut_norm = ttk.Radiobutton(
        cut_box,
        text="Multiple on Single Bed (Normal)",
        variable=var_cut_mode,
        value="normal_multi",
        command=lambda: refresh_all(),
    )
    rb_cut_norm.pack(anchor="w", pady=1)

    rb_cut_diag = ttk.Radiobutton(
        cut_box,
        text="1 Piece per Build Plate (Diagonal)",
        variable=var_cut_mode,
        value="diagonal_single",
        command=lambda: refresh_all(),
    )
    rb_cut_diag.pack(anchor="w", pady=1)

    chk_equal = ttk.Checkbutton(
        cut_box,
        text="Cut into equal-length segments",
        variable=var_equal_cuts,
        command=lambda: refresh_all(),
    )
    chk_equal.pack(anchor="w", pady=(4, 2))

    gap_row = tk.Frame(cut_box, bg="#1b202b")
    gap_row.pack(fill="x", pady=(2, 2))
    tk.Label(gap_row, text="Multi-piece spacing gap (mm):", bg="#1b202b", fg=FG_MUTED, font=("Segoe UI", 8)).pack(
        side="left"
    )
    tk.Entry(
        gap_row,
        textvariable=var_part_gap,
        width=6,
        justify="right",
        bg=BG_INPUT,
        fg=FG_TEXT,
        insertbackground=FG_TEXT,
        relief="flat",
        font=("Consolas", 8),
        highlightbackground=BORDER,
        highlightthickness=1,
    ).pack(side="right")
    var_part_gap.trace_add("write", lambda *_: refresh_all())

    chk_ask_cut = ttk.Checkbutton(
        cut_box,
        text="Ask cut mode confirmation on Save",
        variable=var_ask_on_cut,
    )
    chk_ask_cut.pack(anchor="w", pady=(2, 0))

    export_bar = tk.Frame(sidebar_wrap, bg="#161922", padx=12, pady=10, highlightbackground=BORDER, highlightthickness=1)
    export_bar.grid(row=2, column=0, columnspan=2, sticky="ew")

    dir_row = tk.Frame(export_bar, bg="#161922")
    dir_row.pack(fill="x", pady=(0, 6))
    tk.Label(dir_row, text="Default Folder:", bg="#161922", fg=FG_MUTED, font=("Segoe UI", 8)).pack(side="left")

    def on_pick_folder():
        chosen_dir = filedialog.askdirectory(
            parent=root,
            title="Select Default Output Folder",
            initialdir=var_save_dir.get() or str(downloads_default),
        )
        if chosen_dir:
            var_save_dir.set(chosen_dir)

    btn_folder = tk.Button(
        dir_row,
        text="Folder...",
        command=on_pick_folder,
        bg=BG_CARD,
        fg=FG_TEXT,
        activebackground=BORDER,
        activeforeground=FG_TEXT,
        relief="flat",
        font=("Segoe UI", 8),
        padx=8,
        pady=1,
        cursor="hand2",
    )
    btn_folder.pack(side="right")

    lbl_dir_val = tk.Label(
        export_bar,
        textvariable=var_save_dir,
        bg="#161922",
        fg=ACCENT_BLUE,
        font=("Consolas", 8),
        anchor="w",
    )
    lbl_dir_val.pack(fill="x", pady=(0, 8))

    def ask_cut_mode_modal(plan: dict) -> Optional[str]:
        dlg = tk.Toplevel(root)
        dlg.title("Build Plate Exceeded — Choose Auto-Cut Mode")
        dlg.configure(bg=BG_PANEL)
        dlg.transient(root)
        dlg.grab_set()
        dlg.resizable(False, False)

        dw, dh = 480, 290
        rx = root.winfo_rootx() + (root.winfo_width() - dw) // 2
        ry = root.winfo_rooty() + (root.winfo_height() - dh) // 2
        dlg.geometry(f"{dw}x{dh}+{max(20, rx)}+{max(20, ry)}")

        result: Dict[str, Optional[str]] = {"choice": None}

        tk.Label(
            dlg,
            text="Total Strip Exceeds Diagonal Limit!",
            bg=BG_PANEL,
            fg=ACCENT_AMBER,
            font=("Segoe UI Bold", 11),
        ).pack(anchor="w", padx=18, pady=(14, 4))

        bp_i = plan["bp_info"]
        info_msg = (
            f"Total length ({plan['total_length_mm']:.1f} mm) does not fit Normal ({bp_i['max_normal_mm']:.1f} mm) "
            f"or Diagonal ({bp_i['max_diagonal_mm']:.1f} mm).\n"
            f"How do you want to cut and lay out the pieces?"
        )
        tk.Label(
            dlg,
            text=info_msg,
            bg=BG_PANEL,
            fg=FG_TEXT,
            font=("Segoe UI", 9),
            justify="left",
            wraplength=440,
        ).pack(anchor="w", padx=18, pady=(0, 12))

        n_segs = plan["norm_preview_segs"]
        d_segs = plan["diag_preview_segs"]
        n_desc = (
            f"[1] Multiple on Single Bed — Laying NORMAL\n"
            f"    {len(n_segs)} pieces (max {bp_i['max_normal_mm']:.1f} mm each), packed side-by-side"
            if n_segs
            else "[1] Normal (Unavailable — part wider than bed)"
        )
        d_desc = (
            f"[2] 1 Piece per Build Plate — Laying DIAGONAL\n"
            f"    {len(d_segs)} plates (max {bp_i['max_diagonal_mm']:.1f} mm each), 1 STL per plate"
        )

        def pick(mode_val: str):
            result["choice"] = mode_val
            dlg.destroy()

        btn1 = tk.Button(
            dlg,
            text=n_desc,
            command=lambda: pick("normal_multi"),
            state=("normal" if n_segs else "disabled"),
            bg=BG_CARD,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#0b0f14",
            relief="flat",
            justify="left",
            anchor="w",
            font=("Segoe UI Semibold", 9),
            padx=12,
            pady=8,
            cursor="hand2",
            highlightbackground=ACCENT if var_cut_mode.get() == "normal_multi" else BORDER,
            highlightthickness=1,
        )
        btn1.pack(fill="x", padx=18, pady=(0, 8))

        btn2 = tk.Button(
            dlg,
            text=d_desc,
            command=lambda: pick("diagonal_single"),
            bg=BG_CARD,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#0b0f14",
            relief="flat",
            justify="left",
            anchor="w",
            font=("Segoe UI Semibold", 9),
            padx=12,
            pady=8,
            cursor="hand2",
            highlightbackground=ACCENT if var_cut_mode.get() == "diagonal_single" else BORDER,
            highlightthickness=1,
        )
        btn2.pack(fill="x", padx=18, pady=(0, 10))

        bottom_r = tk.Frame(dlg, bg=BG_PANEL)
        bottom_r.pack(fill="x", padx=18)
        ttk.Checkbutton(
            bottom_r,
            text="Always ask when cutting is required",
            variable=var_ask_on_cut,
        ).pack(side="left")
        tk.Button(
            bottom_r,
            text="Cancel",
            command=dlg.destroy,
            bg=BG_CARD,
            fg=FG_MUTED,
            relief="flat",
            padx=10,
            pady=3,
        ).pack(side="right")

        dlg.wait_window()
        return result["choice"]

    def on_generate_and_save():
        plan = state.get("plan")
        err = state.get("error")
        if err or plan is None:
            messagebox.showerror("Invalid Parameters", str(err or "Please check your input values."), parent=root)
            return

        if plan["needs_cutting"] and var_ask_on_cut.get():
            chosen_cut = ask_cut_mode_modal(plan)
            if chosen_cut is None:
                return
            if chosen_cut != var_cut_mode.get():
                var_cut_mode.set(chosen_cut)
                refresh_all()
                plan = state.get("plan")
                if plan is None:
                    return

        w = plan["strip_width_mm"]
        a = plan["angle_deg"]
        l_tot = plan["total_length_mm"]
        default_fname = f"led_slope_{w:g}mm_{a:g}deg_{l_tot:g}mm.stl"
        init_dir = Path(var_save_dir.get() or str(downloads_default))
        if not init_dir.exists():
            init_dir = downloads_default

        chosen_path = choose_save_path(init_dir, default_fname, parent=root)
        if chosen_path is None:
            return

        var_save_dir.set(str(chosen_path.parent))

        try:
            saved_files = export_plan_to_stls(plan, chosen_path)
            lines = "\n".join(f"• {p.name}  ({lbl})" for p, lbl in saved_files)
            messagebox.showinfo(
                "STL Model(s) Saved!",
                f"Successfully generated {len(saved_files)} STL file(s) in:\n{chosen_path.parent}\n\n{lines}",
                parent=root,
            )
        except Exception as exc:
            messagebox.showerror("Export Failed", f"Could not write STL file:\n{exc}", parent=root)

    btn_save = tk.Button(
        export_bar,
        text="GENERATE & SAVE STL...",
        command=on_generate_and_save,
        bg=ACCENT,
        fg="#081210",
        activebackground=ACCENT_HOVER,
        activeforeground="#081210",
        relief="flat",
        font=("Segoe UI Bold", 10),
        pady=9,
        cursor="hand2",
    )
    btn_save.pack(fill="x")

    right_area = tk.Frame(outer, bg=BG_ROOT)
    right_area.grid(row=0, column=1, sticky="nsew")
    right_area.rowconfigure(0, weight=1, uniform="r")
    right_area.rowconfigure(1, weight=1, uniform="r")
    right_area.columnconfigure(0, weight=1, uniform="c")
    right_area.columnconfigure(1, weight=1, uniform="c")

    panel_xz = tk.Frame(right_area, bg=BG_PANEL, highlightbackground=BORDER, highlightthickness=1)
    panel_xz.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
    panel_xz.rowconfigure(1, weight=1)
    panel_xz.columnconfigure(0, weight=1)

    hdr_xz = tk.Frame(panel_xz, bg=BG_PANEL, padx=10, pady=5)
    hdr_xz.grid(row=0, column=0, sticky="ew")
    tk.Label(
        hdr_xz, text="CROSS-SECTION PROFILE (X–Z)", bg=BG_PANEL, fg=ACCENT, font=("Segoe UI Semibold", 9)
    ).pack(side="left")
    tk.Label(hdr_xz, text="0° Ground Base + Tilted LED Wall", bg=BG_PANEL, fg=FG_MUTED, font=("Segoe UI", 8)).pack(
        side="right"
    )

    cv_xz = tk.Canvas(panel_xz, bg="#141820", highlightthickness=0)
    cv_xz.grid(row=1, column=0, sticky="nsew")

    panel_3d = tk.Frame(right_area, bg=BG_PANEL, highlightbackground=BORDER, highlightthickness=1)
    panel_3d.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
    panel_3d.rowconfigure(1, weight=1)
    panel_3d.columnconfigure(0, weight=1)

    hdr_3d = tk.Frame(panel_3d, bg=BG_PANEL, padx=10, pady=5)
    hdr_3d.grid(row=0, column=0, sticky="ew")
    tk.Label(hdr_3d, text="INTERACTIVE 3D PREVIEW", bg=BG_PANEL, fg=ACCENT, font=("Segoe UI Semibold", 9)).pack(
        side="left"
    )

    def fit_3d_bed():
        state["cam_yaw"] = -0.65
        state["cam_pitch"] = 0.55
        state["cam_zoom"] = 1.0
        state["cam_pan_x"] = 0.0
        state["cam_pan_y"] = 0.0
        state["cam_target"] = (0.0, 0.0, 0.0)
        draw_3d_preview()

    def fit_3d_part():
        plan = state.get("plan")
        if not plan:
            fit_3d_bed()
            return
        bp = plan["bp_info"]
        bed_diag = math.hypot(bp["bp_x"], bp["bp_y"])
        idx = min(int(state["active_plate_idx"]), len(plan["plates"]) - 1)
        pieces = plan["plates"][idx]["pieces"]
        xs = [c[0] for pc in pieces for c in pc["corners_xy"]]
        ys = [c[1] for pc in pieces for c in pc["corners_xy"]]
        span = max(max(xs) - min(xs), max(ys) - min(ys), plan["back_wall_height_mm"] * 2.5, 15.0)
        cx_3d = 0.5 * (min(xs) + max(xs))
        cy_3d = 0.5 * (min(ys) + max(ys))
        cz_3d = plan["back_wall_height_mm"] * 0.45
        state["cam_target"] = (cx_3d, cy_3d, cz_3d)
        state["cam_pan_x"] = 0.0
        state["cam_pan_y"] = 0.0
        state["cam_zoom"] = max(0.8, min(28.0, (bed_diag * 0.85) / span))
        draw_3d_preview()

    def fit_3d_closeup():
        plan = state.get("plan")
        if not plan:
            return
        bp = plan["bp_info"]
        bed_diag = math.hypot(bp["bp_x"], bp["bp_y"])
        idx = min(int(state["active_plate_idx"]), len(plan["plates"]) - 1)
        pc = plan["plates"][idx]["pieces"][0]
        r = pc["rot_z_rad"]
        hl = pc["length_mm"] * 0.5
        fx = hl * math.sin(r) + pc["tx"]
        fy = -hl * math.cos(r) + pc["ty"]
        fz = plan["back_wall_height_mm"] * 0.45
        prof_span = max(plan["strip_width_mm"], plan["back_wall_height_mm"], 8.0) * 3.8
        state["cam_yaw"] = r - 0.62
        state["cam_pitch"] = 0.45
        state["cam_target"] = (fx, fy, fz)
        state["cam_pan_x"] = 0.0
        state["cam_pan_y"] = 0.0
        state["cam_zoom"] = max(2.0, min(32.0, bed_diag / prof_span))
        draw_3d_preview()

    for btn_txt, btn_cmd in (("Full Bed", fit_3d_bed), ("Fit Part", fit_3d_part), ("Close-Up", fit_3d_closeup)):
        tk.Button(
            hdr_3d,
            text=btn_txt,
            command=btn_cmd,
            bg=BG_CARD,
            fg=FG_TEXT,
            activebackground=BORDER,
            activeforeground=ACCENT,
            relief="flat",
            font=("Segoe UI", 8),
            padx=6,
            cursor="hand2",
        ).pack(side="right", padx=(4, 0))

    cv_3d = tk.Canvas(panel_3d, bg="#12161e", highlightthickness=0, cursor="fleur")
    cv_3d.grid(row=1, column=0, sticky="nsew")

    panel_bed = tk.Frame(right_area, bg=BG_PANEL, highlightbackground=BORDER, highlightthickness=1)
    panel_bed.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
    panel_bed.rowconfigure(1, weight=1)
    panel_bed.columnconfigure(0, weight=1)

    hdr_bed = tk.Frame(panel_bed, bg=BG_PANEL, padx=10, pady=5)
    hdr_bed.grid(row=0, column=0, sticky="ew")
    tk.Label(
        hdr_bed,
        text="TOP-DOWN SLICER BUILD PLATE & SAFE BOUNDARY",
        bg=BG_PANEL,
        fg=ACCENT,
        font=("Segoe UI Semibold", 9),
    ).pack(side="left")

    plate_nav = tk.Frame(hdr_bed, bg=BG_PANEL)
    plate_nav.pack(side="right")

    def step_plate(delta: int):
        plan = state.get("plan")
        if not plan:
            return
        n = len(plan["plates"])
        state["active_plate_idx"] = (int(state["active_plate_idx"]) + delta) % n
        draw_bed_preview()
        draw_3d_preview()
        update_plate_nav_label()

    btn_prev_plate = tk.Button(
        plate_nav,
        text="◀ Prev",
        command=lambda: step_plate(-1),
        bg=BG_CARD,
        fg=FG_TEXT,
        relief="flat",
        font=("Segoe UI", 8),
        padx=6,
        cursor="hand2",
    )
    btn_prev_plate.pack(side="left", padx=(0, 6))

    lbl_plate_nav = tk.Label(plate_nav, text="Plate 1 / 1", bg=BG_PANEL, fg=ACCENT_BLUE, font=("Consolas", 9, "bold"))
    lbl_plate_nav.pack(side="left", padx=4)

    btn_next_plate = tk.Button(
        plate_nav,
        text="Next ▶",
        command=lambda: step_plate(1),
        bg=BG_CARD,
        fg=FG_TEXT,
        relief="flat",
        font=("Segoe UI", 8),
        padx=6,
        cursor="hand2",
    )
    btn_next_plate.pack(side="left", padx=(6, 0))

    cv_bed = tk.Canvas(panel_bed, bg="#141820", highlightthickness=0)
    cv_bed.grid(row=1, column=0, sticky="nsew")

    def update_plate_nav_label():
        plan = state.get("plan")
        if not plan:
            lbl_plate_nav.configure(text="—")
            btn_prev_plate.configure(state="disabled")
            btn_next_plate.configure(state="disabled")
            return
        plates = plan["plates"]
        idx = int(state["active_plate_idx"])
        lbl_plate_nav.configure(text=plates[idx]["label"])
        st = "normal" if len(plates) > 1 else "disabled"
        btn_prev_plate.configure(state=st)
        btn_next_plate.configure(state=st)

    def draw_xz_preview(_event=None):
        cv_xz.delete("all")
        w_px = cv_xz.winfo_width()
        h_px = cv_xz.winfo_height()
        if w_px < 40 or h_px < 40:
            return

        plan = state.get("plan")
        if not plan:
            err = state.get("error") or "Invalid input"
            cv_xz.create_text(w_px / 2, h_px / 2, text=str(err), fill=ACCENT_RED, font=("Segoe UI", 9), width=w_px - 30)
            return

        W = plan["strip_width_mm"]
        ang = plan["angle_deg"]
        B = plan["ground_base_width_mm"]
        H = plan["back_wall_height_mm"]
        tip_h = plan["min_base_height_mm"]

        pad = 54
        avail_w = max(40.0, w_px - 2 * pad)
        avail_h = max(40.0, h_px - 2 * pad)

        rad = math.radians(ang)
        ray_len_mm = max(B, H) * 0.35
        nx_ray = -math.sin(rad)
        nz_ray = math.cos(rad)

        min_x = min(0.0, nx_ray * ray_len_mm)
        max_x = max(B, 0.01)
        min_z = 0.0
        max_z = max(H, H * 0.5 + nz_ray * ray_len_mm, 0.01)

        span_x = max(max_x - min_x, 1e-3)
        span_z = max(max_z - min_z, 1e-3)
        scale = min(avail_w / span_x, avail_h / span_z)

        ox = pad + (avail_w - span_x * scale) / 2.0 - min_x * scale
        oz = h_px - pad - (avail_h - span_z * scale) / 2.0

        def to_px(xm: float, zm: float) -> Tuple[float, float]:
            return (ox + xm * scale, oz - zm * scale)

        gx0, gz0 = to_px(min_x - span_x * 0.12, 0.0)
        gx1, gz1 = to_px(max_x + span_x * 0.12, 0.0)
        cv_xz.create_line(gx0, gz0, gx1, gz1, fill="#334155", width=2, dash=(5, 3))

        p_gl = to_px(0.0, 0.0)
        p_gr = to_px(B, 0.0)
        p_sr = to_px(B, H)
        p_sl = to_px(0.0, tip_h)

        pts = [p_gl[0], p_gl[1], p_gr[0], p_gr[1], p_sr[0], p_sr[1]]
        if tip_h > 1e-6:
            pts.extend([p_sl[0], p_sl[1]])

        cv_xz.create_polygon(pts, fill="#22364a", outline=ACCENT_BLUE, width=2)
        cv_xz.create_line(p_gl[0], p_gl[1], p_gr[0], p_gr[1], fill=ACCENT, width=4)
        cv_xz.create_line(p_sl[0], p_sl[1], p_sr[0], p_sr[1], fill=ACCENT_AMBER, width=5)

        for frac in (0.2, 0.5, 0.8):
            sx = B * frac
            sz = tip_h + (H - tip_h) * frac
            ex = sx + nx_ray * ray_len_mm
            ez = sz + nz_ray * ray_len_mm
            ps = to_px(sx, sz)
            pe = to_px(ex, ez)
            cv_xz.create_line(ps[0], ps[1], pe[0], pe[1], fill="#fbbf24", width=1, dash=(3, 3), arrow=tk.LAST)

        arc_r = min(34.0, max(16.0, B * scale * 0.38))
        cv_xz.create_arc(
            p_sl[0] - arc_r,
            p_sl[1] - arc_r,
            p_sl[0] + arc_r,
            p_sl[1] + arc_r,
            start=0,
            extent=ang,
            style=tk.ARC,
            outline=ACCENT_AMBER,
            width=2,
        )
        cv_xz.create_text(
            p_sl[0] + arc_r + 18,
            p_sl[1] - 10,
            text=f"{ang:.1f}°",
            fill=ACCENT_AMBER,
            font=("Consolas", 9, "bold"),
        )

        cv_xz.create_text(
            0.5 * (p_gl[0] + p_gr[0]),
            p_gl[1] + 18,
            text=f"Ground Wall (0°): {B:.2f} mm",
            fill=ACCENT,
            font=("Consolas", 9, "bold"),
        )
        cv_xz.create_text(
            p_gr[0] + 8,
            0.5 * (p_gr[1] + p_sr[1]),
            text=f"H: {H:.2f} mm",
            fill=ACCENT_BLUE,
            font=("Consolas", 8),
            anchor="w",
        )
        mid_sx = 0.5 * (p_sl[0] + p_sr[0]) - 10
        mid_sy = 0.5 * (p_sl[1] + p_sr[1]) - 12
        cv_xz.create_text(
            mid_sx,
            mid_sy,
            text=f"LED Strip: {W:.2f} mm",
            fill="#fde68a",
            font=("Consolas", 9, "bold"),
            anchor="e",
        )

    def draw_bed_preview(_event=None):
        cv_bed.delete("all")
        w_px = cv_bed.winfo_width()
        h_px = cv_bed.winfo_height()
        if w_px < 40 or h_px < 40:
            return

        plan = state.get("plan")
        if not plan:
            err = state.get("error") or "Check inputs"
            cv_bed.create_text(w_px / 2, h_px / 2, text=str(err), fill=ACCENT_RED, font=("Segoe UI", 10))
            return

        bp = plan["bp_info"]
        bp_x, bp_y = bp["bp_x"], bp["bp_y"]
        ux, uy = bp["usable_x"], bp["usable_y"]
        margin = bp["margin_mm"]

        pad = 32
        avail_w = max(40.0, w_px - 2 * pad)
        avail_h = max(40.0, h_px - 2 * pad)
        scale = min(avail_w / bp_x, avail_h / bp_y)

        cx = w_px / 2.0
        cy = h_px / 2.0

        def bed_to_px(x_mm: float, y_mm: float) -> Tuple[float, float]:
            return (cx + x_mm * scale, cy - y_mm * scale)

        tl = bed_to_px(-bp_x / 2.0, bp_y / 2.0)
        br = bed_to_px(bp_x / 2.0, -bp_y / 2.0)
        cv_bed.create_rectangle(tl[0], tl[1], br[0], br[1], fill="#1d222d", outline="#475569", width=2)

        step_mm = 50.0
        gx = -math.floor((bp_x / 2.0) / step_mm) * step_mm
        while gx <= bp_x / 2.0 + 1e-6:
            p0 = bed_to_px(gx, -bp_y / 2.0)
            p1 = bed_to_px(gx, bp_y / 2.0)
            col = "#2d3648" if abs(gx) < 1e-6 else "#242b38"
            cv_bed.create_line(p0[0], p0[1], p1[0], p1[1], fill=col)
            gx += step_mm

        gy = -math.floor((bp_y / 2.0) / step_mm) * step_mm
        while gy <= bp_y / 2.0 + 1e-6:
            p0 = bed_to_px(-bp_x / 2.0, gy)
            p1 = bed_to_px(bp_x / 2.0, gy)
            col = "#2d3648" if abs(gy) < 1e-6 else "#242b38"
            cv_bed.create_line(p0[0], p0[1], p1[0], p1[1], fill=col)
            gy += step_mm

        if margin > 0:
            utl = bed_to_px(-ux / 2.0, uy / 2.0)
            ubr = bed_to_px(ux / 2.0, -uy / 2.0)
            cv_bed.create_rectangle(
                utl[0], utl[1], ubr[0], ubr[1], outline=ACCENT_RED, width=1, dash=(5, 4)
            )
            cv_bed.create_text(
                utl[0] + 6,
                utl[1] + 4,
                text=f"Safe Zone: {ux:g}×{uy:g} mm ({margin:g} mm margin)",
                fill="#f87171",
                font=("Consolas", 8),
                anchor="nw",
            )

        cv_bed.create_text(
            br[0] - 6,
            br[1] - 5,
            text=f"BP: {bp_x:g}×{bp_y:g} mm",
            fill=FG_MUTED,
            font=("Consolas", 8),
            anchor="se",
        )

        idx = min(int(state["active_plate_idx"]), len(plan["plates"]) - 1)
        plate = plan["plates"][idx]
        for pc in plate["pieces"]:
            poly_px = []
            for x_m, y_m in pc["corners_xy"]:
                px, py = bed_to_px(x_m, y_m)
                poly_px.extend([px, py])
            cv_bed.create_polygon(poly_px, fill="#0d4a40", outline=ACCENT, width=2)

            c1 = bed_to_px(*pc["corners_xy"][1])
            c2 = bed_to_px(*pc["corners_xy"][2])
            cv_bed.create_line(c1[0], c1[1], c2[0], c2[1], fill=ACCENT_AMBER, width=2)

            mc = bed_to_px(pc["tx"], pc["ty"])
            cv_bed.create_text(
                mc[0],
                mc[1],
                text=f"{pc['length_mm']:.1f} mm",
                fill="#ffffff",
                font=("Consolas", 8, "bold"),
            )

    # --- 3. Interactive 3D Orbit Preview (Exact Convex Culling + Separating-Plane Sort) ---
    def draw_3d_preview(_event=None):
        cv_3d.delete("all")
        w_px = cv_3d.winfo_width()
        h_px = cv_3d.winfo_height()
        if w_px < 40 or h_px < 40:
            return

        plan = state.get("plan")
        if not plan:
            err = state.get("error") or "Invalid parameters"
            cv_3d.create_text(w_px / 2, h_px / 2, text=str(err), fill=ACCENT_RED, font=("Segoe UI", 9), width=w_px - 30)
            return

        bp = plan["bp_info"]
        bp_x, bp_y = bp["bp_x"], bp["bp_y"]
        ux, uy = bp["usable_x"], bp["usable_y"]

        yaw = float(state["cam_yaw"])
        pitch = float(state["cam_pitch"])
        zoom = float(state["cam_zoom"])
        pan_x = float(state["cam_pan_x"])
        pan_y = float(state["cam_pan_y"])
        tgt_x, tgt_y, tgt_z = state["cam_target"]  # type: ignore[misc]

        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        cpit, spit = math.cos(pitch), math.sin(pitch)

        erx, ery, erz = cyaw, -syaw, 0.0
        eux, euy, euz = syaw * spit, cyaw * spit, cpit
        vx, vy, vz = -syaw * cpit, -cyaw * cpit, spit

        bed_diag = math.hypot(bp_x, bp_y)
        base_scale = (min(w_px, h_px) * 0.72) / max(bed_diag, 1.0) * zoom
        cx = w_px * 0.5 + pan_x
        cy = h_px * 0.54 + pan_y

        def project(pt: Vec3) -> Tuple[float, float]:
            dx = pt[0] - tgt_x
            dy = pt[1] - tgt_y
            dz = pt[2] - tgt_z
            x1 = dx * cyaw - dy * syaw
            y1 = dx * syaw + dy * cyaw
            z2 = y1 * spit + dz * cpit
            return (cx + x1 * base_scale, cy - z2 * base_scale)

        def draw_bed_floor(solid: bool) -> None:
            floor_corners: List[Vec3] = [
                (-bp_x / 2, -bp_y / 2, 0.0),
                (bp_x / 2, -bp_y / 2, 0.0),
                (bp_x / 2, bp_y / 2, 0.0),
                (-bp_x / 2, bp_y / 2, 0.0),
            ]
            f_proj = [project(p) for p in floor_corners]
            f_pts = [coord for p in f_proj for coord in (p[0], p[1])]
            if solid:
                cv_3d.create_polygon(f_pts, fill="#1b212c", outline="#475569", width=2)
            else:
                cv_3d.create_polygon(f_pts, fill="", outline="#334155", width=1, dash=(4, 4))

            step = 50.0
            grid_col = "#252d3d" if solid else "#1e2634"
            gx = -math.floor((bp_x / 2.0) / step) * step
            while gx <= bp_x / 2.0 + 1e-6:
                p0 = project((gx, -bp_y / 2, 0.0))
                p1 = project((gx, bp_y / 2, 0.0))
                cv_3d.create_line(p0[0], p0[1], p1[0], p1[1], fill=grid_col)
                gx += step
            gy = -math.floor((bp_y / 2.0) / step) * step
            while gy <= bp_y / 2.0 + 1e-6:
                p0 = project((-bp_x / 2, gy, 0.0))
                p1 = project((bp_x / 2, gy, 0.0))
                cv_3d.create_line(p0[0], p0[1], p1[0], p1[1], fill=grid_col)
                gy += step

            if bp["margin_mm"] > 0:
                safe_corners: List[Vec3] = [
                    (-ux / 2, -uy / 2, 0.0),
                    (ux / 2, -uy / 2, 0.0),
                    (ux / 2, uy / 2, 0.0),
                    (-ux / 2, uy / 2, 0.0),
                ]
                s_proj = [project(p) for p in safe_corners]
                for i in range(4):
                    a = s_proj[i]
                    b = s_proj[(i + 1) % 4]
                    cv_3d.create_line(a[0], a[1], b[0], b[1], fill=ACCENT_RED, dash=(4, 3))

        kx = 0.34 * erx + 0.46 * eux + 0.82 * vx
        ky = 0.34 * ery + 0.46 * euy + 0.82 * vy
        kz = 0.34 * erz + 0.46 * euz + 0.82 * vz
        k_len = math.sqrt(kx * kx + ky * ky + kz * kz) or 1.0
        kx, ky, kz = kx / k_len, ky / k_len, kz / k_len

        above_bed = pitch >= 0.0
        if above_bed:
            draw_bed_floor(solid=True)

        idx = min(int(state["active_plate_idx"]), len(plan["plates"]) - 1)
        pieces = list(plan["plates"][idx]["pieces"])
        pieces.sort(key=lambda pc: pc["tx"] * vx + pc["ty"] * vy)

        for pc in pieces:
            faces = build_piece_convex_faces(
                strip_width=plan["strip_width_mm"],
                angle_deg=plan["angle_deg"],
                extrude_length=pc["length_mm"],
                min_base_height=plan["min_base_height_mm"],
                rot_z_rad=pc["rot_z_rad"],
                tx=pc["tx"],
                ty=pc["ty"],
            )
            for verts, (nx, ny, nz), role in faces:
                facing = nx * vx + ny * vy + nz * vz
                if facing <= 1e-6:
                    continue

                proj_pts = [project(v) for v in verts]
                flat_coords = [c for p in proj_pts for c in p]

                key_dot = max(0.0, nx * kx + ny * ky + nz * kz)
                sky_dot = 0.5 + 0.5 * nz
                intensity = 0.34 + 0.50 * key_dot + 0.16 * sky_dot

                if role == "led_slope":
                    r = int(min(255, 248 * intensity))
                    g = int(min(255, 170 * intensity))
                    b = int(min(255, 36 * intensity))
                    edge_col = "#fcd34d"
                elif role == "ground":
                    r = int(min(255, 18 * intensity))
                    g = int(min(255, 214 * intensity))
                    b = int(min(255, 158 * intensity))
                    edge_col = "#34d399"
                elif role == "cap":
                    r = int(min(255, 42 * intensity))
                    g = int(min(255, 186 * intensity))
                    b = int(min(255, 202 * intensity))
                    edge_col = "#2dd4bf"
                else:
                    r = int(min(255, 22 * intensity))
                    g = int(min(255, 162 * intensity))
                    b = int(min(255, 146 * intensity))
                    edge_col = "#0d9488"

                fill_hex = f"#{r:02x}{g:02x}{b:02x}"
                cv_3d.create_polygon(flat_coords, fill=fill_hex, outline=edge_col, width=1)

        if not above_bed:
            draw_bed_floor(solid=False)

        gx0, gy0 = 32.0, h_px - 28.0
        axis_len = 18.0
        for ax_vec, ax_col, ax_lbl in (
            ((1.0, 0.0, 0.0), "#ef4444", "X"),
            ((0.0, 1.0, 0.0), "#22c55e", "Y"),
            ((0.0, 0.0, 1.0), "#38bdf8", "Z"),
        ):
            sx_a = gx0 + (ax_vec[0] * erx + ax_vec[1] * ery + ax_vec[2] * erz) * axis_len
            sy_a = gy0 - (ax_vec[0] * eux + ax_vec[1] * euy + ax_vec[2] * euz) * axis_len
            cv_3d.create_line(gx0, gy0, sx_a, sy_a, fill=ax_col, width=2)
            cv_3d.create_text(
                sx_a + (sx_a - gx0) * 0.35,
                sy_a + (sy_a - gy0) * 0.35,
                text=ax_lbl,
                fill=ax_col,
                font=("Consolas", 7, "bold"),
            )

        cv_3d.create_rectangle(10, 10, 21, 21, fill="#d9941e", outline="#fcd34d")
        cv_3d.create_text(26, 15, text="LED Strip Wall", fill=FG_TEXT, font=("Segoe UI", 8), anchor="w")
        if not above_bed:
            cv_3d.create_rectangle(115, 10, 126, 21, fill="#10b981", outline="#34d399")
            cv_3d.create_text(
                131, 15, text="0° Ground Base (Underneath View)", fill=ACCENT, font=("Segoe UI", 8), anchor="w"
            )
        cv_3d.create_text(
            w_px - 10,
            h_px - 10,
            text="L-Drag: Orbit • R-Drag: Pan • Scroll: Zoom",
            fill=FG_MUTED,
            font=("Segoe UI", 7),
            anchor="se",
        )

    def on_3d_press(event):
        cv_3d.focus_set()
        state["drag_last"] = (event.x, event.y)
        state["drag_mode"] = "orbit"

    def on_3d_pan_press(event):
        cv_3d.focus_set()
        state["drag_last"] = (event.x, event.y)
        state["drag_mode"] = "pan"

    def on_3d_motion(event):
        last = state.get("drag_last")
        if not last:
            return
        dx = event.x - last[0]
        dy = event.y - last[1]
        state["drag_last"] = (event.x, event.y)

        if state.get("drag_mode") == "pan":
            state["cam_pan_x"] = float(state["cam_pan_x"]) + dx
            state["cam_pan_y"] = float(state["cam_pan_y"]) + dy
        else:
            state["cam_yaw"] = float(state["cam_yaw"]) + dx * 0.012
            new_pitch = float(state["cam_pitch"]) + dy * 0.012
            state["cam_pitch"] = max(-1.48, min(1.53, new_pitch))
        draw_3d_preview()

    def on_3d_wheel(event):
        factor = 1.15 if event.delta > 0 else (1.0 / 1.15)
        old_zoom = float(state["cam_zoom"])
        new_zoom = max(0.30, min(35.0, old_zoom * factor))
        if abs(new_zoom - old_zoom) > 1e-6:
            w_px = cv_3d.winfo_width()
            h_px = cv_3d.winfo_height()
            cx = w_px * 0.5 + float(state["cam_pan_x"])
            cy = h_px * 0.54 + float(state["cam_pan_y"])
            ratio = new_zoom / old_zoom
            state["cam_pan_x"] = float(state["cam_pan_x"]) - (event.x - cx) * (ratio - 1.0)
            state["cam_pan_y"] = float(state["cam_pan_y"]) - (event.y - cy) * (ratio - 1.0)
            state["cam_zoom"] = new_zoom
            draw_3d_preview()

    cv_3d.bind("<ButtonPress-1>", on_3d_press)
    cv_3d.bind("<ButtonPress-2>", on_3d_pan_press)
    cv_3d.bind("<ButtonPress-3>", on_3d_pan_press)
    cv_3d.bind("<B1-Motion>", on_3d_motion)
    cv_3d.bind("<B2-Motion>", on_3d_motion)
    cv_3d.bind("<B3-Motion>", on_3d_motion)
    cv_3d.bind("<Enter>", lambda _e: cv_3d.bind_all("<MouseWheel>", on_3d_wheel))
    cv_3d.bind("<Leave>", lambda _e: cv_3d.unbind_all("<MouseWheel>"))

    cv_xz.bind("<Configure>", draw_xz_preview)
    cv_bed.bind("<Configure>", draw_bed_preview)
    cv_3d.bind("<Configure>", draw_3d_preview)

    def refresh_all():
        try:
            w = float(var_width.get())
            a = float(var_angle.get())
            l_tot = float(var_length.get())
            base_h = float(var_base.get() or "0")
            bp_x = float(var_bp_x.get())
            bp_y = float(var_bp_y.get())
            margin = float(var_margin.get() or "0")
            gap = float(var_part_gap.get() or "4")

            plan = plan_build_plate_layout(
                strip_width=w,
                angle_deg=a,
                total_length=l_tot,
                min_base_height=base_h,
                bp_x=bp_x,
                bp_y=bp_y,
                margin_mm=margin,
                pref_mode=var_pref_mode.get(),
                cut_mode=var_cut_mode.get(),
                equal_cuts=var_equal_cuts.get(),
                part_gap_mm=gap,
            )
            state["plan"] = plan
            state["error"] = None
            if int(state["active_plate_idx"]) >= len(plan["plates"]):
                state["active_plate_idx"] = 0

            lbl_geom_stats.configure(
                fg=ACCENT_BLUE,
                text=(
                    f"Ground Base (0°): {plan['ground_base_width_mm']:.2f} mm\n"
                    f"Back Wall Height: {plan['back_wall_height_mm']:.2f} mm"
                ),
            )

            bpi = plan["bp_info"]
            lbl_bp_limits.configure(
                fg=FG_MUTED,
                text=(
                    f"Usable Bed  : {bpi['usable_x']:.1f} × {bpi['usable_y']:.1f} mm\n"
                    f"Max Normal  : {bpi['max_normal_mm']:.1f} mm\n"
                    f"Max Diagonal: {bpi['max_diagonal_mm']:.1f} mm ({bpi['diag_rot_z_deg']:.1f}°)"
                ),
            )

            lvl = plan["status_level"]
            if lvl == "ok":
                lbl_status_badge.configure(bg="#13332b", fg=ACCENT, text=f"✔ {plan['status_text']}")
                cut_box.configure(highlightbackground=BORDER)
                lbl_cut_header.configure(fg=FG_MUTED, text="When Cutting Is Needed (currently fits 1 pc):")
            elif lvl == "info":
                lbl_status_badge.configure(bg="#152c3d", fg=ACCENT_BLUE, text=f"➜ {plan['status_text']}")
                cut_box.configure(highlightbackground=BORDER)
                lbl_cut_header.configure(fg=FG_MUTED, text="When Cutting Is Needed (auto-diagonal active):")
            else:
                lbl_status_badge.configure(bg="#3b2610", fg=ACCENT_AMBER, text=f"✂ {plan['status_text']}")
                cut_box.configure(highlightbackground=ACCENT_AMBER)
                lbl_cut_header.configure(fg=ACCENT_AMBER, text="✂ Auto-Cut Active — Choose Cut Layout:")

            n_cnt = len(plan["norm_preview_segs"])
            d_cnt = len(plan["diag_preview_segs"])
            rb_cut_norm.configure(
                text=f"Multiple on Single Bed — Normal ({n_cnt} pcs)" if n_cnt else "Normal (too wide)"
            )
            rb_cut_diag.configure(text=f"1 per Build Plate — Diagonal ({d_cnt} plates)")

        except Exception as exc:
            state["plan"] = None
            state["error"] = str(exc)
            lbl_status_badge.configure(bg="#3b151a", fg=ACCENT_RED, text=f"⚠ {exc}")

        update_plate_nav_label()
        draw_xz_preview()
        draw_bed_preview()
        draw_3d_preview()

    state["ready"] = True
    refresh_all()
    root.mainloop()


# =====================================================================
# 3. CLI ENTRYPOINT (LAUNCHES GUI WHEN RUN WITH NO CLI FLAGS)
# =====================================================================

def run_cli(args: argparse.Namespace) -> None:
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(errors="replace")
        except Exception:
            pass
    interactive = args.width is None and args.angle is None

    strip_width = args.width
    if strip_width is None:
        strip_width = float(input("Enter LED strip width (mm) [e.g. 10]: ").strip() or "10")

    angle_deg = args.angle
    if angle_deg is None:
        angle_deg = float(input("Enter slope angle (degrees, 0-90) [e.g. 30]: ").strip() or "30")

    total_length = args.length
    if total_length is None:
        if interactive:
            raw_len = input("Enter total desired mount length (mm) [default: 100]: ").strip()
            total_length = float(raw_len) if raw_len else 100.0
        else:
            total_length = 100.0

    bp_str = args.bp
    if bp_str is None and interactive:
        bp_str = input("Enter your Build Plate (BP) size in mm (e.g. 256 or 220x220) [default: 256]: ").strip() or "256"
    elif bp_str is None:
        bp_str = "256"

    margin_mm = args.margin
    if margin_mm is None and interactive:
        raw_margin = input("Enter safe boundary margin from bed edges in mm [default: 5]: ").strip()
        margin_mm = float(raw_margin) if raw_margin else 5.0
    elif margin_mm is None:
        margin_mm = 5.0

    bp_x, bp_y = parse_bp_size(bp_str)
    base_width = strip_width * math.cos(math.radians(angle_deg))
    bp_info = compute_bp_limits(bp_x, bp_y, base_width, margin_mm=margin_mm)

    print(
        f"\n[Build Plate {bp_x:g}x{bp_y:g} mm | Safe Boundary: {margin_mm:g} mm/edge -> Usable: {bp_info['usable_x']:g}x{bp_info['usable_y']:g} mm]\n"
        f"Max single-piece length -> Normal: {bp_info['max_normal_mm']:.1f} mm | Diagonal: {bp_info['max_diagonal_mm']:.1f} mm"
    )

    pref_mode = args.mode
    if pref_mode is None and interactive:
        raw_mode = input("Choose printing orientation — [1] Normal (default) or [2] Diagonal: ").strip().lower()
        pref_mode = "diagonal" if raw_mode in ("2", "d", "diag", "diagonal") else "normal"
    elif pref_mode is None:
        pref_mode = "normal"

    probe = plan_build_plate_layout(
        strip_width=strip_width,
        angle_deg=angle_deg,
        total_length=total_length,
        min_base_height=args.base,
        bp_x=bp_x,
        bp_y=bp_y,
        margin_mm=margin_mm,
        pref_mode=pref_mode,
        cut_mode=args.cut_mode or "normal_multi",
        equal_cuts=args.equal_cuts,
    )
    print(f"-> {probe['status_text']}")

    cut_mode = args.cut_mode or "normal_multi"
    equal_cuts = args.equal_cuts

    if probe["needs_cutting"] and args.cut_mode is None and interactive:
        print("\nHow would you like to cut and arrange the pieces?")
        print(
            f"  [1] Multiple on single bed laying NORMAL "
            f"({len(probe['norm_preview_segs'])} pieces, max {bp_info['max_normal_mm']:.1f} mm each, packed side-by-side)"
        )
        print(
            f"  [2] 1 per build plate laying DIAGONAL    "
            f"({len(probe['diag_preview_segs'])} pieces, max {bp_info['max_diagonal_mm']:.1f} mm each, 1 STL per plate)"
        )
        choice = input("Select cut mode [1/2, default: 1]: ").strip().lower()
        cut_mode = "diagonal_single" if choice in ("2", "d", "diag", "diagonal") else "normal_multi"

        if not args.equal_cuts:
            eq_choice = input(
                "Cut sizing — [1] Max length pieces + remainder (default) or [2] Equal-length pieces: "
            ).strip()
            if eq_choice == "2":
                equal_cuts = True

    plan = plan_build_plate_layout(
        strip_width=strip_width,
        angle_deg=angle_deg,
        total_length=total_length,
        min_base_height=args.base,
        bp_x=bp_x,
        bp_y=bp_y,
        margin_mm=margin_mm,
        pref_mode=pref_mode,
        cut_mode=cut_mode,
        equal_cuts=equal_cuts,
    )

    downloads_dir = Path.home() / "Downloads"
    default_filename = f"led_slope_{strip_width:g}mm_{angle_deg:g}deg_{total_length:g}mm.stl"

    if args.output:
        out_candidate = Path(args.output)
        base_output_path = out_candidate if out_candidate.is_absolute() else (downloads_dir / out_candidate)
    elif args.no_dialog:
        base_output_path = downloads_dir / default_filename
    else:
        print("\nOpening file manager to choose where to save the STL file(s)...")
        chosen = choose_save_path(downloads_dir, default_filename)
        if chosen is None:
            print("Save cancelled.")
            return
        base_output_path = chosen

    saved_files = export_plan_to_stls(plan, base_output_path)

    print("\n--- LED Slope 3D Model Summary ---")
    print(f"Sloped LED wall width : {plan['strip_width_mm']:.2f} mm (at {plan['angle_deg']:.1f} deg)")
    print(f"Ground base width (X) : {plan['ground_base_width_mm']:.2f} mm (at 0 deg)")
    print(f"Back wall height  (Z) : {plan['back_wall_height_mm']:.2f} mm")
    print(f"Total target length   : {total_length:.2f} mm")
    print(
        f"Build Plate size      : {bp_x:g} x {bp_y:g} mm "
        f"(Safe margin: {margin_mm:g} mm/edge -> Usable: {bp_info['usable_x']:g} x {bp_info['usable_y']:g} mm)"
    )
    print("Generated STL file(s) :")
    for fpath, desc in saved_files:
        print(f"  * {fpath}  [{desc}]")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="3D LED Strip Slope Studio — launches GUI when run with no flags, or runs via CLI flags."
    )
    parser.add_argument("--gui", action="store_true", help="Launch the interactive 3D Studio GUI (default)")
    parser.add_argument("--cli", action="store_true", help="Run in interactive terminal CLI mode instead of GUI")
    parser.add_argument("-w", "--width", type=float, help="Width of the LED strip / sloped wall in mm")
    parser.add_argument("-a", "--angle", type=float, help="Tilt angle in degrees (0-90)")
    parser.add_argument("-l", "--length", type=float, default=None, help="Total desired length in mm")
    parser.add_argument("--bp", type=str, default=None, help="Build plate size in mm (e.g. '256' or '250x210')")
    parser.add_argument("-m", "--margin", type=float, default=None, help="Safe boundary margin in mm (default: 5.0)")
    parser.add_argument("--mode", choices=["normal", "diagonal"], default=None, help="Preferred orientation")
    parser.add_argument(
        "--cut-mode",
        choices=["normal_multi", "diagonal_single"],
        default=None,
        help="Cut mode when exceeding build plate",
    )
    parser.add_argument("--equal-cuts", action="store_true", help="Cut into equal-length segments")
    parser.add_argument("-b", "--base", type=float, default=0.0, help="Minimum tip thickness in mm (default: 0.0)")
    parser.add_argument("-o", "--output", type=str, default=None, help="Direct output STL path")
    parser.add_argument("--no-dialog", action="store_true", help="Skip Save As dialog and save to Downloads")

    args = parser.parse_args()

    has_cli_params = any(
        [
            args.cli,
            args.width is not None,
            args.angle is not None,
            args.length is not None,
            args.bp is not None,
            args.output is not None,
        ]
    )
    if args.gui or not has_cli_params:
        launch_gui()
    else:
        run_cli(args)


if __name__ == "__main__":
    main()
