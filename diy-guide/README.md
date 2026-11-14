# PocketPCR: picture guide, 3D assembly video and India notes

An **unofficial**, beginner-friendly companion to the GaudiLabs PocketPCR kit, made for DIY builders in India.

| File | What it is |
|---|---|
| [`pocketpcr-picture-guide-india.pdf`](pocketpcr-picture-guide-india.pdf) | 12-page printable guide: parts list, screws at actual size, 10 numbered assembly steps, loading samples, screen map, troubleshooting, buying in India (₹), notes for every ⚠ flag |
| [`pocketpcr-assembly-3d.mp4`](pocketpcr-assembly-3d.mp4) | 86-second 3D animation of the assembly steps with captions |
| [`SOURCING-INDIA.md`](SOURCING-INDIA.md) | Where to get the boards made (with parts soldered on) and the loose parts, with rough ₹ costs |
| [`source/pdf/`](source/pdf/) | Python (ReportLab) script that draws the PDF |
| [`source/video/`](source/video/) | three.js scene + Playwright script that renders the video frames |

Print the PDF at **100 % / actual size** so the screw drawings on page 4 match real screws.

## Before you trust it

- The official instructions PDF (gaudi.ch) could not be downloaded when this was made. The guide is based on a CC BY 4.0 copy of the official text (*Instrucciones para la PocketPCR*, N. A. Méndez & U. Gaudenz, 2023) plus the KiCad layout and BOM in this repo. **The photos in the original PDF were not seen.**
- Anything we had to infer (which fan screws take nuts vs spacers, fan airflow direction, lid stack order, etc.) is marked ⚠ N1–N10 in the PDF, with an explanation on the last page.
- ₹ prices are rough estimates from November 2026. Check before buying.
- Firmware fixes are in a separate pull request (V1.03). The guide's troubleshooting page refers to them.

## Rebuilding

**PDF**

```sh
pip install reportlab
python3 source/pdf/build_guide.py pocketpcr-picture-guide-india.pdf
```

Needs the DejaVu Sans fonts (`fonts-dejavu` on Debian/Ubuntu). Set `FONT_DIR` if they live somewhere else.

**Video**

```sh
cd source/video
npm install                      # three.js + playwright
npx playwright install chromium  # only if you don't already have a Playwright browser
python3 -m http.server 8765 &    # ES modules need http://, not file://
mkdir frames
node render.js http://localhost:8765/index.html frames 24
ffmpeg -framerate 24 -i frames/f%05d.jpg -c:v libx264 -pix_fmt yuv420p -crf 22 pocketpcr-assembly-3d.mp4
```

Open `http://localhost:8765/index.html` and call `renderAt(seconds)` in the browser console to preview any moment.

## Licence

Text and drawings: CC BY 4.0, like the official instructions they are based on. Code: GPL-3.0, like the rest of this repository. PocketPCR is designed by GaudiLabs.
