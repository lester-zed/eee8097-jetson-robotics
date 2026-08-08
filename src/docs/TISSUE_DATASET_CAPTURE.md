# Tissue-pack dataset capture

Run inside Docker:

```bash
cd /workspace/src
chmod +x run_tissue_capture.sh
./run_tissue_capture.sh
```

Controls:
- SPACE or p: save a positive `tissue_pack` image.
- n: save a negative/background image.
- q or ESC: quit.

Default output:

```text
/workspace/data/tissue_pack/raw/
├── positive/
├── negative/
└── capture_manifest.csv
```

Headless automatic positive capture:

```bash
./run_tissue_capture.sh --no-preview --auto positive --interval 1.0 --count 200
```

Headless automatic negative capture:

```bash
./run_tissue_capture.sh --no-preview --auto negative --interval 1.0 --count 50
```

If needed, choose the camera explicitly:

```bash
./run_tissue_capture.sh --camera /dev/video0
```
