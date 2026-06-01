# Static Book Spine Recognition

This folder packages the still-image workflow into one place.

## Contents

- `static_recognition_gui.py`
- `capture_static_recognition.py`

## Recommended use

For distant shelves or low-feature scenes, use:

- `HYBRID`
- `1920 x 1080`
- threshold `30`
- `min_matches = 6`
- `min_inliers = 5`

## Launch the GUI

```bash
python3 "/home/boreas/project/435surf(1)/435surf/static_recognition_tool/static_recognition_gui.py"
```

## Run a one-shot capture

```bash
python3 "/home/boreas/project/435surf(1)/435surf/static_recognition_tool/capture_static_recognition.py" \
  --algorithm hybrid \
  --width 1920 --height 1080 \
  --threshold 30 \
  --min-matches 6 \
  --min-inliers 5
```

## Output files

The folder writes its captured images here by default:

- `static_capture.jpg`
- `static_recognition.jpg`
- `static_capture_gui.jpg`
- `static_recognition_gui.jpg`

