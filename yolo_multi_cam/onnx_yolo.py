"""YOLO (ultralytics export) inference with onnxruntime — no ultralytics/torch.

Desktop backend for ``multi_cam_yolo``: export once with

    yolo export model=yolo11n.pt format=onnx dynamic=True simplify=True

and point ``model_path`` at the ``.onnx``. Runs on the TensorRT EP (FP16)
when available, else CUDA, else CPU. Input is letterboxed to a fixed
``imgsz`` square so TensorRT builds a single engine. Graphs from
``tools/fuse_onnx.py`` (uint8 input, top-K output) are detected and skip the
CPU blob conversion and per-anchor postprocessing.
"""

import ast
import ctypes
import glob
import os
import site
import sys

import cv2
import numpy as np

_PROVIDERS = {
    "tensorrt": "TensorrtExecutionProvider",
    "cuda": "CUDAExecutionProvider",
    "cpu": "CPUExecutionProvider",
}
_preloaded = False


def preload_gpu_libs():
    """dlopen CUDA / cuDNN / TensorRT libs from pip wheels on sys.path.

    onnxruntime-gpu does not find pip-installed CUDA libraries by itself.
    """
    global _preloaded
    if _preloaded:
        return
    _preloaded = True
    roots = {d for d in sys.path if d.endswith("site-packages")}
    try:
        roots.update(site.getsitepackages())
    except AttributeError:
        pass
    pats = ["nvidia/*/lib/lib*.so*", "nvidia/cu*/lib/lib*.so*", "tensorrt_libs/lib*.so*"]
    pending = sorted({p for r in roots for pat in pats for p in glob.glob(os.path.join(r, pat))})
    for _ in range(4):  # dependency order agnostic
        failed = []
        for lib in pending:
            try:
                ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                failed.append(lib)
        if not failed or len(failed) == len(pending):
            break
        pending = failed


class OnnxYolo:
    def __init__(self, model_path, providers=("tensorrt", "cuda", "cpu"), imgsz=640,
                 fp16=True, cache_dir="", device_id=0):
        preload_gpu_libs()
        import onnxruntime as ort

        available = set(ort.get_available_providers())
        prov = []
        for alias in providers:
            name = _PROVIDERS.get(str(alias).lower(), alias)
            if name not in available:
                continue
            if name == "TensorrtExecutionProvider":
                opts = {"device_id": device_id, "trt_fp16_enable": fp16,
                        "trt_timing_cache_enable": True}
                if cache_dir:
                    os.makedirs(cache_dir, exist_ok=True)
                    opts.update(trt_engine_cache_enable=True, trt_engine_cache_path=cache_dir,
                                trt_timing_cache_path=cache_dir)
                prov.append((name, opts))
            elif name == "CUDAExecutionProvider":
                prov.append((name, {"device_id": device_id}))
            else:
                prov.append((name, {}))
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self.session = ort.InferenceSession(model_path, so,
                                            providers=prov or ["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.imgsz = int(imgsz)
        meta = self.session.get_modelmeta().custom_metadata_map
        # tools/fuse_onnx.py graph: uint8 NHWC BGR in, top-K boxes out (pre/post
        # processing on the GPU)
        self.fused = self.session.get_inputs()[0].type == "tensor(uint8)"
        self.fused_classes = ast.literal_eval(meta.get("fused_classes", "[]")) if self.fused else []
        try:
            self.names = {int(k): v for k, v in ast.literal_eval(meta.get("names", "{}")).items()}
        except (ValueError, SyntaxError):
            self.names = {}

    @property
    def active_providers(self):
        return self.session.get_providers()

    def prepare(self, bgr):
        """Letterbox one BGR frame -> (canvas, meta). Thread-safe, so the node
        runs it in its decode pool next to cv2.imdecode."""
        h, w = bgr.shape[:2]
        s = self.imgsz / max(h, w)
        nw, nh = int(round(w * s)), int(round(h * s))
        px, py = (self.imgsz - nw) // 2, (self.imgsz - nh) // 2
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, np.uint8)
        canvas[py:py + nh, px:px + nw] = cv2.resize(bgr, (nw, nh),
                                                    interpolation=cv2.INTER_LINEAR)
        return canvas, (s, px, py, w, h)

    def predict(self, frames, conf=0.25, iou=0.5, classes=None, pad_to=0):
        """frames: list of BGR images; pad_to: pad the batch to this size
        (a fixed input shape keeps TensorRT from rebuilding engines).

        Returns one (N, 6) float32 array per frame: x1, y1, x2, y2, score, class
        in pixels of the original image.
        """
        return self.predict_prepared([self.prepare(f) for f in frames], conf, iou,
                                     classes, pad_to)

    def predict_prepared(self, items, conf=0.25, iou=0.5, classes=None, pad_to=0):
        """Like predict() for the output of prepare()."""
        canvases = [c for c, _ in items]
        canvases += [canvases[0]] * max(0, pad_to - len(canvases))
        if self.fused:
            boxes, scores, cls = self.session.run(None, {self.input_name: np.stack(canvases)})
            dets = [(boxes[i], scores[i], cls[i]) for i in range(len(items))]
        else:
            x = cv2.dnn.blobFromImages(canvases, 1 / 255.0, swapRB=True)  # NCHW RGB float32
            out = self.session.run(None, {self.input_name: x})[0][:len(items)]
            dets = [self._best_class(pred, classes) for pred in out]
        results = []
        for (cxcywh, sc, cl), (_, (s, px, py, w, h)) in zip(dets, items):
            keep = sc >= conf
            if classes and self.fused and not set(classes) >= set(self.fused_classes):
                keep &= np.isin(cl, classes)
            if not keep.any():
                results.append(np.zeros((0, 6), np.float32))
                continue
            cxcywh, sc, cl = cxcywh[keep], sc[keep], cl[keep].astype(np.int64)
            xyxy = np.empty_like(cxcywh)
            xyxy[:, :2] = cxcywh[:, :2] - cxcywh[:, 2:] / 2
            xyxy[:, 2:] = cxcywh[:, :2] + cxcywh[:, 2:] / 2
            xywh = np.concatenate([xyxy[:, :2], cxcywh[:, 2:]], 1)
            idx = cv2.dnn.NMSBoxesBatched(xywh.tolist(), sc.tolist(), cl.tolist(), conf, iou)
            idx = np.asarray(idx, dtype=np.int64).reshape(-1)
            xyxy = xyxy[idx]
            xyxy[:, [0, 2]] = np.clip((xyxy[:, [0, 2]] - px) / s, 0, w)
            xyxy[:, [1, 3]] = np.clip((xyxy[:, [1, 3]] - py) / s, 0, h)
            results.append(np.concatenate(
                [xyxy, sc[idx, None], cl[idx, None].astype(np.float32)], 1).astype(np.float32))
        return results

    @staticmethod
    def _best_class(pred, classes):
        """Raw [4+nc, anchors] head output -> (cxcywh, score, class) per anchor."""
        pred = pred.T                                  # (anchors, 4 + nc)
        scores = pred[:, 4:]
        if classes:
            mask = np.zeros(scores.shape[1], bool)
            mask[[c for c in classes if c < scores.shape[1]]] = True
            scores = np.where(mask, scores, 0.0)
        cls = scores.argmax(1)
        return pred[:, :4], scores[np.arange(len(cls)), cls], cls
