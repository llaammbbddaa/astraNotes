## Overview

This package is a ROS2 drone-image detection stack: it watches a folder of camera images, runs object detection, annotates the image with bounding boxes, and publishes results over ROS topics.

The main runtime code lives here:

- `object_detection.py`
- `object_detection_sahi.py`
- `new_od.py`
- `setup.py`

---

## What each program does

### 1) `object_detection.py`
This is the older “general-purpose” detection node.

What it does:
- Creates a ROS node named `object_detection_node`
- Watches a camera feed directory for new JPG/PNG files
- Loads YOLO, SAHI, and MobileNet models
- Tries:
  1. SAHI slicing for small objects
  2. direct YOLO detection
  3. MobileNet validation
  4. OpenCV fallback for people/tent-like objects
- Draws boxes on the image and saves annotated output
- Publishes:
  - `/detection_results` as an image
  - `/detection_info` as a string message

Major packages/tools:
- ROS2 `rclpy`
- `sensor_msgs`, `std_msgs`
- `cv_bridge`
- OpenCV (`cv2`)
- NumPy
- PyTorch
- Ultralytics YOLO
- SAHI
- TensorFlow + MobileNetV3
- PIL for image handling

This is essentially a mixed “best effort” detector: it tries a few models and falls back gracefully.

---

### 2) `object_detection_sahi.py`
This is the dedicated small-object detector.

What it does:
- Creates a ROS node named `sahi_object_detection_node`
- Looks for images in a folder like `mapping_photos`
- Uses SAHI to split large images into overlapping slices
- Runs YOLO on each slice to detect small objects
- Merges overlapping detections and filters them by area/aspect/confidence
- Focuses on `person` / `tent` / `object` classification
- Publishes annotated images and stats

This is the most “mission-specific” detector in the project: it is tuned for small tents and people in aerial imagery.

Major packages/tools:
- ROS2 `rclpy`, `Node`
- `cv_bridge`
- OpenCV
- NumPy
- SAHI (`AutoDetectionModel`, `get_sliced_prediction`)
- Ultralytics YOLO
- PyTorch
- `vision_msgs`, `mavros_msgs`, `std_srvs`
- GPU cleanup / Jetson tuning utilities

It contains a lot of parameter tuning to balance:
- slice size
- overlap
- confidence threshold
- area filtering
- GPU memory handling

---

### 3) `new_od.py`
This is the newer lifecycle-based ROS node.

What it does:
- Uses a ROS lifecycle node instead of a plain node
- Loads a model in a background thread
- Supports auto model resolution:
  - `.engine` TensorRT model preferred
  - falls back to `.pt` model if needed
- Optimizes GPU settings for Jetson/Orin hardware
- Watches `camera_feed_path` for new images
- Runs SAHI inference through the shared processor
- Saves best crops and annotated output
- Publishes detection results to `/sahi_detection_results` and `/image_detection`

This is the most production-oriented version of the detection pipeline in the package.

Major packages/tools:
- ROS2 lifecycle API (`LifecycleNode`, `State`, `TransitionCallbackReturn`)
- `cv_bridge`
- OpenCV
- NumPy
- PyTorch
- Ultralytics YOLO
- SAHI
- TensorRT
- Jetson-specific helpers (`nvpmodel`, `jetson_clocks` checks)
- ROS message types: `Image`, `String`, `ImageResult`

---

### 4) `detection_processor.py`
This is not a ROS node by itself; it is the shared SAHI detection helper.

What it does:
- Accepts a frame and model
- Converts BGR to RGB for SAHI
- Runs `get_sliced_prediction(...)`
- Reviews each predicted box
- Classifies detections as:
  - `person`
  - `tent`
  - `object`
- Rejects bad detections based on:
  - confidence
  - area
  - aspect ratio
  - false-positive filtering
- Runs periodic CUDA cleanup

Major packages/tools:
- OpenCV
- NumPy
- PyTorch
- SAHI
- custom classification rules

This file is the logic layer behind the “sensible” filtering.

---

### 5) `model_manager.py`
This is the model-loader and resolver layer.

What it does:
- Finds the ROS workspace path
- Resolves model paths relative to the workspace
- Checks for `.engine` and `.pt` files
- Optionally converts PyTorch model to TensorRT
- Loads a SAHI-compatible model
- Warms up the model before inference
- Handles GPU/CPU selection

Major packages/tools:
- PyTorch
- Ultralytics YOLO
- SAHI
- TensorRT
- Python `os`, `pathlib`, `typing`

This is the “where do I load the model from?” file.

---

### 6) `annotation.py`
This is the visual annotation utility.

What it does:
- Draws colored boxes around objects
- Labels them as `PERSON` or `TENT`
- Adds an image header with counts and processing time
- Saves crop images of the strongest detections

Major packages/tools:
- OpenCV
- NumPy

---

### 7) `gpu_utils.py`
This is the hardware-optimization helper.

What it does:
- Detects whether CUDA is available
- Chooses CPU vs GPU
- Adjusts slice size and overlap to avoid GPU OOM
- Checks Jetson power mode and warns if it is not in max-performance mode
- Performs CUDA cache cleanup

Major packages/tools:
- PyTorch
- OS / subprocess
- Jetson runtime checks

---

### 8) `check_gpu.py`
This is a diagnostic script.

What it does:
- Prints whether Python, PyTorch, CUDA, cuDNN, and GPU are installed
- Checks device count and memory
- Helps diagnose environment problems before running the model

Major packages/tools:
- PyTorch
- CUDA / GPU tooling

---

### 9) `YOLOxSAHI`
This folder is a reference/test implementation.

Files:
- `main.py`
- `main_fast.py`
- `process_video.py`

What they do:
- Run YOLO + SAHI directly on images or video
- Useful as a prototype/reference for the ROS package
- One version is faster, another is more general

Major packages/tools:
- OpenCV
- PyTorch
- Ultralytics YOLO
- SAHI

---

## Main ROS entry points

The package registers these console scripts in `setup.py`:

- `object_detection_sahi = detection.object_detection_sahi:main`
- `new_od = detection.new_od:main`

So the main runtime commands are effectively:
- `object_detection_sahi`
- `new_od`

---

## Major packages / tools used across the package

- ROS2 + `rclpy`
- OpenCV (`cv2`)
- NumPy
- PyTorch
- Ultralytics YOLO
- SAHI (small-object detection)
- TensorRT (for optimized model inference in `new_od`)
- TensorFlow + MobileNetV3 (older node)
- `cv_bridge` for ROS/OpenCV conversion
- `sensor_msgs`, `std_msgs`, `vision_msgs`, `mavros_msgs`
- Jetson hardware checks and GPU memory optimization

> In short: this is a ROS2 object-detection package built around YOLO and SAHI, tuned for small aerial targets such as people and tents, with extra GPU/Jetson optimization layers.

[[understanding deprecated object detection stack]]