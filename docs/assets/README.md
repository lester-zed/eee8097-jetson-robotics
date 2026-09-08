# README media assets

This directory is reserved for portfolio-facing media used by the repository root `README.md`.

## Recommended files

```text
docs/assets/demo.gif
docs/assets/hardware_overview.jpg
```

Optional additions:

```text
docs/assets/system_architecture.png
docs/assets/calibration_result.png
```

## Demo GIF

For a resume-facing GitHub repository, keep the main demo short and immediately understandable:

- 8–15 seconds is ideal.
- Show the camera view / target detection first, then the robot response.
- Crop away unrelated desktop UI where possible.
- 720p–960 px width is usually enough for README display.
- Prefer a reasonably compressed GIF rather than committing a large MP4 to the repository.

A simple conversion with `ffmpeg`:

```bash
ffmpeg -i demo.mp4 \
  -vf "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer" \
  -loop 0 demo.gif
```

Then copy the result to:

```text
docs/assets/demo.gif
```

## Hardware image

Use one clear landscape photograph showing the complete setup. A 1200–1800 px wide JPEG is sufficient for README use.

Recommended filename:

```text
docs/assets/hardware_overview.jpg
```

If useful, add a second annotated system image or architecture figure instead of several similar photographs.

## Enable the media in the root README

The root `README.md` already contains a commented media block near the top. After both files are committed, remove the surrounding `<!--` and `-->` lines so GitHub renders:

```html
<p align="center">
  <img src="docs/assets/demo.gif" alt="Vision-guided robotic manipulation demo" width="900">
</p>

<p align="center">
  <img src="docs/assets/hardware_overview.jpg" alt="Jetson Orin Nano robotics hardware setup" width="900">
</p>
```

For a clickable full video, keep the GIF as the README preview and link it to a GitHub Release, GitHub-hosted video attachment, or external video page rather than storing a large MP4 in the main Git history.

## Commit example

```bash
git checkout docs/portfolio-readme-cleanup
mkdir -p docs/assets
cp /path/to/demo.gif docs/assets/demo.gif
cp /path/to/hardware_overview.jpg docs/assets/hardware_overview.jpg

git add docs/assets/demo.gif docs/assets/hardware_overview.jpg README.md
git commit -m "docs: add hardware demo media"
git push
```
