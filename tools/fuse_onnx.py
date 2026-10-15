#!/usr/bin/env python3
"""Move YOLO pre/post-processing into the ONNX graph (runs in TensorRT).

Turns an ultralytics detect export (``images`` float NCHW RGB 0..1 ->
``output0`` [N, 4+nc, anchors]) into

    images_u8  uint8 [N, S, S, 3]  letterboxed BGR, as produced by cv2
    -> boxes   float [N, K, 4]     cx, cy, w, h in letterbox pixels
       scores  float [N, K]        best score over the kept classes
       classes float [N, K]        class id

with the top-K anchors per image. The CPU then only letterboxes (in the
decode threads) and runs NMS on K boxes instead of transposing and
arg-maxing 8400 x 84 floats per image, and the host->device copy shrinks
4x (uint8 instead of float32). On the AGX Orin (6 cameras batched) this
raised YOLO from ~11 Hz to ~22 Hz per ArkCam (ZED at its full 14 Hz).

    python3 tools/fuse_onnx.py ~/ros2_ws/models/onnx/yolo11n.onnx \
        -o ~/ros2_ws/models/onnx/yolo11n_person_fused.onnx --classes 0

``--classes`` bakes a class filter into the graph (the node's
``filter_classes`` must then be a subset of it).
"""

import argparse

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def fuse(src, dst, classes, topk, imgsz):
    m = onnx.load(src)
    g = m.graph
    assert len(g.input) == 1 and len(g.output) == 1, "expected a plain detect export"
    old_in, old_out = g.input[0].name, g.output[0].name

    def const(name, arr):
        g.initializer.append(numpy_helper.from_array(np.asarray(arr), name))
        return name

    # --- pre: uint8 NHWC BGR -> float NCHW RGB / 255 -------------------------
    pre = [
        helper.make_node("Cast", ["images_u8"], ["_f"], to=TensorProto.FLOAT),
        helper.make_node("Gather", ["_f", const("_bgr2rgb", np.array([2, 1, 0], np.int64))],
                         ["_rgb"], axis=3),
        helper.make_node("Transpose", ["_rgb"], ["_nchw"], perm=[0, 3, 1, 2]),
        helper.make_node("Mul", ["_nchw", const("_inv255", np.array(1 / 255.0, np.float32))],
                         [old_in]),
    ]

    # --- post: [N, 4+nc, A] -> top-K boxes / scores / classes ----------------
    post = [
        helper.make_node("Slice", [old_out, const("_s0", [0]), const("_s4", [4]),
                                   const("_ax1", [1])], ["_box_t"]),
        helper.make_node("Transpose", ["_box_t"], ["_box"], perm=[0, 2, 1]),   # N,A,4
        helper.make_node("Slice", [old_out, const("_s4b", [4]), const("_sinf", [1 << 30]),
                                   const("_ax1b", [1])], ["_cls_all"]),       # N,nc,A
    ]
    score_src = "_cls_all"
    if classes:
        post.append(helper.make_node(
            "Gather", ["_cls_all", const("_keep_cls", np.array(classes, np.int64))],
            ["_cls_keep"], axis=1))
        score_src = "_cls_keep"
    post += [
        helper.make_node("ReduceMax", [score_src], ["_best"], axes=[1], keepdims=0),  # N,A
        helper.make_node("ArgMax", [score_src], ["_arg"], axis=1, keepdims=0),        # N,A
        helper.make_node("TopK", ["_best", const("_k", np.array([topk], np.int64))],
                         ["scores", "_idx"], axis=1),                                 # N,K
        helper.make_node("GatherElements", ["_arg", "_idx"], ["_arg_k"], axis=1),
        helper.make_node("Cast", ["_arg_k"], ["_arg_kf"], to=TensorProto.FLOAT),
    ]
    if classes:
        # local index -> real class id
        post.append(helper.make_node(
            "Gather", [const("_cls_ids", np.array(classes, np.float32)), "_arg_k"],
            ["classes"], axis=0))
    else:
        post.append(helper.make_node("Identity", ["_arg_kf"], ["classes"]))
    post += [
        helper.make_node("Unsqueeze", ["_idx", const("_ax2", [2])], ["_idx3"]),
        helper.make_node("Shape", ["_box"], ["_box_shape"]),
        helper.make_node("Slice", ["_box_shape", const("_z", [0]), const("_one", [1])],
                         ["_n"]),
        helper.make_node("Concat", ["_n", const("_k4", np.array([topk, 4], np.int64))],
                         ["_exp_shape"], axis=0),
        helper.make_node("Expand", ["_idx3", "_exp_shape"], ["_idx_exp"]),
        helper.make_node("GatherElements", ["_box", "_idx_exp"], ["boxes"], axis=1),
    ]

    nodes = pre + list(g.node) + post
    del g.node[:]
    g.node.extend(nodes)
    del g.input[:]
    g.input.append(helper.make_tensor_value_info(
        "images_u8", TensorProto.UINT8, ["batch", imgsz, imgsz, 3]))
    del g.output[:]
    g.output.extend([
        helper.make_tensor_value_info("boxes", TensorProto.FLOAT, ["batch", topk, 4]),
        helper.make_tensor_value_info("scores", TensorProto.FLOAT, ["batch", topk]),
        helper.make_tensor_value_info("classes", TensorProto.FLOAT, ["batch", topk]),
    ])
    del g.value_info[:]
    meta = {p.key: p.value for p in m.metadata_props}
    meta.update(fused="1", fused_classes=repr(list(classes)), imgsz=f"[{imgsz}, {imgsz}]")
    del m.metadata_props[:]
    for k, v in meta.items():
        m.metadata_props.add(key=k, value=v)
    onnx.checker.check_model(m)
    onnx.save(m, dst)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--classes", type=int, nargs="*", default=[],
                    help="class ids to keep (default: all)")
    ap.add_argument("--topk", type=int, default=300)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()
    fuse(a.model, a.output, a.classes, a.topk, a.imgsz)
    print(f"wrote {a.output}")


if __name__ == "__main__":
    main()
