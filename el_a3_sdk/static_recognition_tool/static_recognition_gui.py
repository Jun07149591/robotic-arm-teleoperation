from __future__ import annotations

import threading
import time
import sys
import tkinter as tk
from argparse import Namespace
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from book_spine_core import DEFAULT_TEMPLATE_PATH, format_result_summary
from capture_static_recognition import capture_frame, recognize
from cnn_backup import DEFAULT_CNN_MODEL_PATH


class StaticRecognitionApp:
    ALGORITHMS = ("HYBRID", "SIFT", "ORB", "CNN", "SURF")

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Static Book Spine Recognition")
        self.root.geometry("1380x860")
        self.root.minsize(1180, 760)

        self.template_path = Path(DEFAULT_TEMPLATE_PATH)
        self.model_path = Path(DEFAULT_CNN_MODEL_PATH)
        self.busy = False
        self.raw_photo = None
        self.annotated_photo = None

        self.algorithm_var = tk.StringVar(value="HYBRID")
        self.source_var = tk.StringVar(value="realsense")
        self.threshold_var = tk.DoubleVar(value=30.0)
        self.cnn_threshold_var = tk.DoubleVar(value=0.45)
        self.width_var = tk.IntVar(value=1920)
        self.height_var = tk.IntVar(value=1080)
        self.warmup_var = tk.IntVar(value=30)
        self.min_matches_var = tk.IntVar(value=6)
        self.min_inliers_var = tk.IntVar(value=5)
        self.template_var = tk.StringVar(value=str(self.template_path))
        self.model_var = tk.StringVar(value=str(self.model_path))
        self.status_var = tk.StringVar(value="Ready")
        self.metrics_var = tk.StringVar(value="No capture yet")

        self._build_ui()

    def _build_ui(self) -> None:
        controls = ttk.Frame(self.root, padding=10)
        controls.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(controls, text="Algorithm").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            controls,
            values=self.ALGORITHMS,
            textvariable=self.algorithm_var,
            state="readonly",
            width=9,
        ).grid(row=0, column=1, padx=(8, 16), sticky="w")

        ttk.Label(controls, text="Source").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.source_var, width=14).grid(row=0, column=3, padx=(8, 16), sticky="w")

        ttk.Label(controls, text="Size").grid(row=0, column=4, sticky="w")
        ttk.Entry(controls, textvariable=self.width_var, width=6).grid(row=0, column=5, padx=(8, 4), sticky="w")
        ttk.Label(controls, text="x").grid(row=0, column=6, sticky="w")
        ttk.Entry(controls, textvariable=self.height_var, width=6).grid(row=0, column=7, padx=(4, 16), sticky="w")

        ttk.Label(controls, text="Threshold").grid(row=0, column=8, sticky="w")
        ttk.Entry(controls, textvariable=self.threshold_var, width=6).grid(row=0, column=9, padx=(8, 16), sticky="w")

        ttk.Label(controls, text="Warmup").grid(row=0, column=10, sticky="w")
        ttk.Entry(controls, textvariable=self.warmup_var, width=5).grid(row=0, column=11, padx=(8, 16), sticky="w")

        self.capture_button = ttk.Button(controls, text="Capture + Recognize", command=self.capture_and_recognize)
        self.capture_button.grid(row=0, column=12, padx=(8, 0), sticky="w")

        ttk.Label(controls, text="Matches").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(controls, textvariable=self.min_matches_var, width=5).grid(row=1, column=1, padx=(8, 16), sticky="w", pady=(8, 0))

        ttk.Label(controls, text="Inliers").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(controls, textvariable=self.min_inliers_var, width=5).grid(row=1, column=3, padx=(8, 16), sticky="w", pady=(8, 0))

        ttk.Button(controls, text="Template", command=self.choose_template).grid(row=1, column=4, sticky="w", pady=(8, 0))
        ttk.Label(controls, textvariable=self.template_var).grid(row=1, column=5, columnspan=5, sticky="w", pady=(8, 0))

        ttk.Button(controls, text="Model", command=self.choose_model).grid(row=1, column=10, sticky="w", pady=(8, 0))
        ttk.Label(controls, textvariable=self.model_var).grid(row=1, column=11, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Label(controls, textvariable=self.status_var).grid(row=2, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Label(controls, textvariable=self.metrics_var).grid(row=2, column=5, columnspan=8, sticky="w", pady=(8, 0))

        preview = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        preview.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        raw_frame = ttk.Frame(preview)
        annotated_frame = ttk.Frame(preview)
        preview.add(raw_frame, weight=1)
        preview.add(annotated_frame, weight=1)

        ttk.Label(raw_frame, text="Raw capture").pack(side=tk.TOP, anchor="w")
        ttk.Label(annotated_frame, text="Recognition result").pack(side=tk.TOP, anchor="w")

        self.raw_label = ttk.Label(raw_frame, anchor="center")
        self.raw_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))
        self.annotated_label = ttk.Label(annotated_frame, anchor="center")
        self.annotated_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(8, 0))

    def choose_template(self) -> None:
        path = filedialog.askopenfilename(
            title="Select template image",
            filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.tif *.tiff *.webp"), ("All files", "*.*")],
        )
        if path:
            self.template_path = Path(path)
            self.template_var.set(str(self.template_path))

    def choose_model(self) -> None:
        path = filedialog.askopenfilename(title="Select CNN model", filetypes=[("ONNX files", "*.onnx"), ("All files", "*.*")])
        if path:
            self.model_path = Path(path)
            self.model_var.set(str(self.model_path))

    def capture_and_recognize(self) -> None:
        if self.busy:
            return
        self.busy = True
        self.capture_button.config(state=tk.DISABLED)
        self.status_var.set("Capturing...")
        self.metrics_var.set("")
        thread = threading.Thread(target=self._capture_worker, daemon=True)
        thread.start()

    def _args(self) -> Namespace:
        return Namespace(
            source=self.source_var.get().strip() or "realsense",
            template=self.template_path,
            model=self.model_path,
            algorithm=self.algorithm_var.get().lower(),
            threshold=float(self.threshold_var.get()),
            cnn_threshold=float(self.cnn_threshold_var.get()),
            min_matches=int(self.min_matches_var.get()),
            min_inliers=int(self.min_inliers_var.get()),
            width=int(self.width_var.get()),
            height=int(self.height_var.get()),
            fps=30,
            warmup_frames=int(self.warmup_var.get()),
            raw_output=SCRIPT_DIR / "static_capture_gui.jpg",
            annotated_output=SCRIPT_DIR / "static_recognition_gui.jpg",
        )

    def _capture_worker(self) -> None:
        try:
            args = self._args()
            started = time.perf_counter()
            backend, frame = capture_frame(args)
            cv2.imwrite(str(args.raw_output), frame)
            annotated, result = recognize(args, frame)
            cv2.imwrite(str(args.annotated_output), annotated)
            elapsed = time.perf_counter() - started
            self.root.after(0, self._show_result, backend, frame, annotated, result, elapsed)
        except Exception as exc:
            self.root.after(0, self._show_error, exc)

    def _show_result(self, backend, frame, annotated, result, elapsed: float) -> None:
        self._set_image(self.raw_label, frame, "raw")
        self._set_image(self.annotated_label, annotated, "annotated")
        self.status_var.set(f"Captured from {backend} in {elapsed:.1f}s")
        self.metrics_var.set(format_result_summary(result))
        self.busy = False
        self.capture_button.config(state=tk.NORMAL)

    def _show_error(self, exc: Exception) -> None:
        self.status_var.set("Capture failed")
        self.metrics_var.set(str(exc))
        self.busy = False
        self.capture_button.config(state=tk.NORMAL)
        messagebox.showerror("Static recognition error", str(exc))

    def _set_image(self, label: ttk.Label, frame_bgr, attr_name: str) -> None:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        max_width = max(label.winfo_width(), 540)
        max_height = max(label.winfo_height(), 420)
        scale = min(max_width / image.width, max_height / image.height, 1.0)
        if scale < 1.0:
            image = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(image=image)
        setattr(self, f"{attr_name}_photo", photo)
        label.configure(image=photo)


def main() -> None:
    root = tk.Tk()
    StaticRecognitionApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

