#!/usr/bin/env python3
import argparse
import json
import math
import os

import cv2
import numpy as np
import rosbag
import yaml


def load_board_spec(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    marker_ids = data.get("marker_ids", {})
    required_ids = sorted({int(v) for v in marker_ids.values()})
    dictionary_name = str(data["dictionary"])
    return {
        "board_name": data.get("board_name", "board"),
        "dictionary_name": dictionary_name,
        "required_ids": required_ids,
    }


def get_aruco_dictionary(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module not available")
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError("unknown aruco dictionary %s" % dictionary_name)
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def detect_markers(detector, frame):
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _rejected = detector.detectMarkers(frame)
    else:
        corners, ids, _rejected = cv2.aruco.detectMarkers(frame, detector["dictionary"], parameters=detector["params"])
    if ids is None:
        return [], []
    return corners, [int(v) for v in ids.flatten().tolist()]


def build_detector(dictionary):
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, params)
    return {"dictionary": dictionary, "params": params}


def decode_bgr(msg):
    if msg.encoding == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
    if msg.encoding == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if msg.encoding == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if msg.encoding == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    raise ValueError("unsupported image encoding %s" % msg.encoding)


def quantile(values, q):
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=np.float64), q))


def min_max(values):
    if not values:
        return None, None
    return float(min(values)), float(max(values))


def bucket_3(value):
    if value < 1.0 / 3.0:
        return "low"
    if value < 2.0 / 3.0:
        return "mid"
    return "high"


def analyze_bag(bag_path, image_topic, board_spec, frame_stride):
    dictionary = get_aruco_dictionary(board_spec["dictionary_name"])
    detector = build_detector(dictionary)
    required_ids = set(board_spec["required_ids"])

    total_frames = 0
    analyzed_frames = 0
    any_detection_frames = 0
    two_or_more_required_frames = 0
    all_required_frames = 0
    per_id_counts = {marker_id: 0 for marker_id in required_ids}
    partial_bbox_width_fracs = []
    partial_bbox_height_fracs = []
    partial_center_x_fracs = []
    partial_center_y_fracs = []
    partial_scale_metric_fracs = []
    partial_center_bins = set()
    partial_scale_bins = set()
    bbox_width_fracs = []
    bbox_height_fracs = []
    center_x_fracs = []
    center_y_fracs = []
    scale_metric_fracs = []
    center_bins = set()
    scale_bins = set()
    first_stamp = None
    last_stamp = None

    with rosbag.Bag(bag_path, "r") as bag:
        for _topic, msg, stamp in bag.read_messages(topics=[image_topic]):
            total_frames += 1
            last_stamp = stamp
            if first_stamp is None:
                first_stamp = stamp
            if frame_stride > 1 and ((total_frames - 1) % frame_stride) != 0:
                continue
            analyzed_frames += 1
            frame = decode_bgr(msg)
            corners, ids = detect_markers(detector, frame)
            if ids:
                any_detection_frames += 1
            ids_set = set(ids)
            for marker_id in required_ids:
                if marker_id in ids_set:
                    per_id_counts[marker_id] += 1

            by_id = {}
            for marker_id, corner in zip(ids, corners):
                by_id[int(marker_id)] = corner.reshape((4, 2))
            visible_required_ids = sorted(required_ids.intersection(ids_set))
            if len(visible_required_ids) >= 2:
                two_or_more_required_frames += 1
                partial_pts = np.concatenate([by_id[marker_id] for marker_id in visible_required_ids], axis=0)
                partial_min_xy = np.min(partial_pts, axis=0)
                partial_max_xy = np.max(partial_pts, axis=0)
                partial_width_frac = float((partial_max_xy[0] - partial_min_xy[0]) / float(frame.shape[1]))
                partial_height_frac = float((partial_max_xy[1] - partial_min_xy[1]) / float(frame.shape[0]))
                partial_center_x = float((partial_min_xy[0] + partial_max_xy[0]) * 0.5 / float(frame.shape[1]))
                partial_center_y = float((partial_min_xy[1] + partial_max_xy[1]) * 0.5 / float(frame.shape[0]))
                partial_scale_metric = max(partial_width_frac, partial_height_frac)

                partial_bbox_width_fracs.append(partial_width_frac)
                partial_bbox_height_fracs.append(partial_height_frac)
                partial_center_x_fracs.append(partial_center_x)
                partial_center_y_fracs.append(partial_center_y)
                partial_scale_metric_fracs.append(partial_scale_metric)
                partial_center_bins.add((bucket_3(partial_center_x), bucket_3(partial_center_y)))
                if partial_scale_metric < 0.35:
                    partial_scale_bins.add("small")
                elif partial_scale_metric < 0.55:
                    partial_scale_bins.add("medium")
                else:
                    partial_scale_bins.add("large")

            if not required_ids.issubset(ids_set):
                continue
            all_required_frames += 1

            pts = np.concatenate([by_id[marker_id] for marker_id in sorted(required_ids)], axis=0)
            min_xy = np.min(pts, axis=0)
            max_xy = np.max(pts, axis=0)
            width_frac = float((max_xy[0] - min_xy[0]) / float(frame.shape[1]))
            height_frac = float((max_xy[1] - min_xy[1]) / float(frame.shape[0]))
            center_x = float((min_xy[0] + max_xy[0]) * 0.5 / float(frame.shape[1]))
            center_y = float((min_xy[1] + max_xy[1]) * 0.5 / float(frame.shape[0]))
            scale_metric = max(width_frac, height_frac)

            bbox_width_fracs.append(width_frac)
            bbox_height_fracs.append(height_frac)
            center_x_fracs.append(center_x)
            center_y_fracs.append(center_y)
            scale_metric_fracs.append(scale_metric)
            center_bins.add((bucket_3(center_x), bucket_3(center_y)))
            if scale_metric < 0.35:
                scale_bins.add("small")
            elif scale_metric < 0.55:
                scale_bins.add("medium")
            else:
                scale_bins.add("large")

    duration_sec = None
    if first_stamp is not None and last_stamp is not None:
        duration_sec = float((last_stamp - first_stamp).to_sec())

    summary = {
        "bag_path": bag_path,
        "image_topic": image_topic,
        "board_name": board_spec["board_name"],
        "dictionary_name": board_spec["dictionary_name"],
        "required_ids": sorted(required_ids),
        "total_frames": int(total_frames),
        "analyzed_frames": int(analyzed_frames),
        "frame_stride": int(frame_stride),
        "duration_sec": duration_sec,
        "any_detection_frames": int(any_detection_frames),
        "two_or_more_required_frames": int(two_or_more_required_frames),
        "two_or_more_required_ratio": 0.0 if analyzed_frames == 0 else float(two_or_more_required_frames) / float(analyzed_frames),
        "all_required_frames": int(all_required_frames),
        "all_required_ratio": 0.0 if analyzed_frames == 0 else float(all_required_frames) / float(analyzed_frames),
        "per_id_counts": per_id_counts,
        "partial_bbox_width_frac_min": min_max(partial_bbox_width_fracs)[0],
        "partial_bbox_width_frac_max": min_max(partial_bbox_width_fracs)[1],
        "partial_bbox_height_frac_min": min_max(partial_bbox_height_fracs)[0],
        "partial_bbox_height_frac_max": min_max(partial_bbox_height_fracs)[1],
        "partial_bbox_width_frac_q50": quantile(partial_bbox_width_fracs, 0.5),
        "partial_bbox_height_frac_q50": quantile(partial_bbox_height_fracs, 0.5),
        "partial_center_x_frac_min": min_max(partial_center_x_fracs)[0],
        "partial_center_x_frac_max": min_max(partial_center_x_fracs)[1],
        "partial_center_y_frac_min": min_max(partial_center_y_fracs)[0],
        "partial_center_y_frac_max": min_max(partial_center_y_fracs)[1],
        "partial_scale_metric_min": min_max(partial_scale_metric_fracs)[0],
        "partial_scale_metric_max": min_max(partial_scale_metric_fracs)[1],
        "partial_occupied_center_bins": sorted(["%s_%s" % item for item in partial_center_bins]),
        "partial_occupied_scale_bins": sorted(partial_scale_bins),
        "bbox_width_frac_min": min_max(bbox_width_fracs)[0],
        "bbox_width_frac_max": min_max(bbox_width_fracs)[1],
        "bbox_height_frac_min": min_max(bbox_height_fracs)[0],
        "bbox_height_frac_max": min_max(bbox_height_fracs)[1],
        "bbox_width_frac_q50": quantile(bbox_width_fracs, 0.5),
        "bbox_height_frac_q50": quantile(bbox_height_fracs, 0.5),
        "center_x_frac_min": min_max(center_x_fracs)[0],
        "center_x_frac_max": min_max(center_x_fracs)[1],
        "center_y_frac_min": min_max(center_y_fracs)[0],
        "center_y_frac_max": min_max(center_y_fracs)[1],
        "scale_metric_min": min_max(scale_metric_fracs)[0],
        "scale_metric_max": min_max(scale_metric_fracs)[1],
        "occupied_center_bins": sorted(["%s_%s" % item for item in center_bins]),
        "occupied_scale_bins": sorted(scale_bins),
        "usable_minimum": bool(two_or_more_required_frames >= 40 and len(partial_center_bins) >= 3 and len(partial_scale_bins) >= 2),
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="Audit marker visibility and pose coverage in a calibration bag.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--board-spec", required=True)
    parser.add_argument("--frame-stride", type=int, default=5)
    parser.add_argument("--summary-json", default="")
    args = parser.parse_args()

    summary = analyze_bag(args.bag, args.image_topic, load_board_spec(args.board_spec), max(1, args.frame_stride))
    if args.summary_json:
        os.makedirs(os.path.dirname(args.summary_json), exist_ok=True)
        with open(args.summary_json, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)

    print("[summary] %s" % json.dumps(summary, sort_keys=True))
    if not summary["all_required_frames"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
