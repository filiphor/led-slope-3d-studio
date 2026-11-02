# LED Slope 3D Studio

**Created by Filip Hor (`Filiposhor`)** — [@filiphor](https://github.com/filiphor)  
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC_BY_4.0-brightgreen.svg)](https://creativecommons.org/licenses/by/4.0/)

A zero-dependency Python desktop studio & CLI generator that creates watertight, 3D-printable `.stl` sloped wedges/mounts for tilting LED strips at any exact angle, with **automatic build-plate fitting, diagonal fallback, safe-boundary margins, and multi-piece auto-cutting**.

---

## Features

- **Exact LED Slope Geometry:**
  - Enter your **LED Strip Width** ($W$ in `mm`) and **Tilt Angle** ($\theta$ in degrees).
  - The leftmost ($X = 0$) and rightmost ($X = W \cos\theta$) points of the tilted LED wall project straight down to the ground ($0^\circ$, $Z = 0$), forming the flat mounting base you stick to your wall, desk, or aluminum profile.
  - Optional **Minimum Tip Thickness** (`mm`) for 3D printer nozzle durability at low angles.
- **Full Dark-Mode 3D Studio GUI:**
  - **Cross-Section Profile (X–Z):** Live blueprint showing the $0^\circ$ ground base, tilted LED strip wall, angle arc, and light throw direction.
  - **Interactive 3D Bed Preview:** Click-and-drag to orbit in 3D, right-drag to pan, scroll wheel to zoom, with directional shading.
  - **Top-Down Slicer Build Plate View:** Shows your build plate grid, dashed **Safe Boundary** zone, and exact piece footprints.
- **Smart Build Plate Auto-Fit & Auto-Cut:**
  - Built-in printer presets (`256×256` Bambu X1/P1/A1, `180×180` A1 Mini, `220×220` Ender/K1, `250×210` Prusa, `300×300`, `350×350`) or custom `X × Y` dimensions.
  - Customizable **Safe Boundary Margin** (default `5 mm` from each build plate edge).
  - Choose between **Normal** or **Diagonal** printing:
    1. If **Normal** fits $\rightarrow$ exports 1 piece aligned normally.
    2. If **Normal** doesn't fit $\rightarrow$ **automatically checks Diagonal** and rotates the model to fit as 1 single piece if possible.
    3. If **Diagonal** doesn't fit $\rightarrow$ prompts to auto-cut into either:
       - **Multiple pieces on a single bed laying Normal** (packed side-by-side into one `.stl` per bed), or
       - **1 piece per build plate laying Diagonal** (1 `.stl` per plate).

---

## Quick Start

### 1. Run the Standalone Windows `.exe` (No Python Required)
Download **`LED_Slope_Studio.exe`** from the [**Releases**](https://github.com/filiphor/led-slope-3d-studio/releases) page and double-click it.

### 2. Run from Python Source (Zero External Dependencies)
Requires only standard Python 3 (uses built-in `tkinter`, `math`, `struct`):

```bash
# Launch the interactive 3D Studio GUI
python led_slope_generator.py
```
Or double-click **`LED_Slope_Studio.pyw`** on Windows to open the GUI without a terminal window.

### 3. Command-Line (CLI) Usage
```bash
# Interactive terminal mode
python led_slope_generator.py --cli

# Direct CLI flags (e.g., 10mm strip at 30 deg, 600mm total length on a 256x256 bed with 5mm margin)
python led_slope_generator.py -w 10 -a 30 -l 600 --bp 256 --margin 5 --mode normal --cut-mode normal_multi
```

---

## License & Commercial Attribution (`CC BY 4.0`)

This project is licensed under the **Creative Commons Attribution 4.0 International License ([CC BY 4.0](LICENSE))**.

You are free to use, share, modify, and use this tool or its generated 3D models **commercially (including selling prints or software)**, **provided that you visibly credit the creator**:

> **Created by Filip Hor (`Filiposhor`)** — https://github.com/filiphor/led-slope-3d-studio
