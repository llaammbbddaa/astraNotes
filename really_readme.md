# ASTRA Camera System: the classroom guide

This guide explains the project as if we were learning it together in a high school engineering class. You do not need to have used a robot, a neural network, or ROS before. We will start with the purpose of the system, follow a picture through the software, and then look inside the code.

This document describes the implementation in this repository as of **October 29, 2026**. Example measurements are illustrations, not benchmarks of the physical camera. Commands assume you are in the repository root unless a step explicitly puts you inside a container.

We focus on **`zr10_ros2`**, which controls the camera and publishes video images, and **`yolo26_ros`**, which detects objects. We also explain **`astra_geotagging`**, because image location is part of the larger project. Its automatic connection to camera images is still incomplete.

## Contents

1. [What are we building?](#1-what-are-we-building)
2. [The vocabulary, from scratch](#2-the-vocabulary-from-scratch)
3. [The system map](#3-the-system-map)
4. [Repository tour](#4-repository-tour)
5. [First experiment: video without a camera](#5-first-experiment-video-without-a-camera)
6. [Camera control](#6-camera-control)
7. [Photo downloading](#7-photo-downloading)
8. [Turning video into ROS images](#8-turning-video-into-ros-images)
9. [The image message contract](#9-the-image-message-contract)
10. [The live image detector](#10-the-live-image-detector)
11. [Object detection mathematics](#11-object-detection-mathematics)
12. [Tiling](#12-tiling)
13. [Understanding the outputs](#13-understanding-the-outputs)
14. [The folder detector](#14-the-folder-detector)
15. [The original video feeder](#15-the-original-video-feeder)
16. [The standalone CLI](#16-the-standalone-cli)
17. [Launch files](#17-launch-files)
18. [Docker and Compose](#18-docker-and-compose)
19. [Native builds](#19-native-builds)
20. [GPS and the missing connections](#20-gps-and-the-missing-connections)
21. [Command cookbook](#21-command-cookbook)
22. [Performance and tradeoffs](#22-performance-and-tradeoffs)
23. [Testing](#23-testing)
24. [Troubleshooting](#24-troubleshooting)
25. [Extending the project](#25-extending-the-project)
26. [Classroom exercises](#26-classroom-exercises)
27. [Reference and glossary](#27-reference-and-glossary)

## 1. What are we building?

Imagine an aircraft carrying a camera. A computer receives pictures and tries to answer: is there a person, where does that person appear in the picture, and can we save an image with a box around them? Eventually we also want to associate an image with the aircraft's location when it was captured.

These questions involve different jobs. Sending a zoom command is a control job. Reading video is a media job. Recognizing a car is a machine-learning job. Matching a timestamp to GPS is a timing and location job. Separate programs make these jobs easier to test and replace.

The current implementation can:

- Build all three ROS packages in a ROS 2 Humble Docker image.
- Exercise camera-control callbacks in dry-run mode or send implemented commands over UDP.
- Download JPEGs through the implemented camera HTTP media API when enabled.
- Decode an RTSP stream or local video through OpenCV and FFmpeg.
- Publish selected decoded frames as ROS image messages.
- Run YOLO26 inference using OpenCV DNN on the CPU.
- Publish annotated images and save JPEG/CSV output pairs.
- Run a separate directory-watching detection pipeline.
- Match supplied capture times to buffered GPS samples and write metadata to a configured JPEG.

Software checks have passed for the Docker build, model inference, topic processing, local video publication, and combined launch. **Physical ZR10 operations and streaming still need device validation.**

What is not implemented yet matters too:

- There is no camera acknowledgment receiver or state-feedback loop in the UDP driver.
- Detection does not track the same object across frames.
- Detection boxes are image coordinates, not map coordinates.
- There is no structured bounding-box ROS output; results are annotations and CSV files.
- GPS matching is not automatically connected to streaming frames or their detections.
- Stream timestamps describe receipt/decode time, not hardware exposure time.
- Installing the software on a Jetson does not automatically enable GPU inference.
- This is not an autonomous flight, navigation, or target-following system.

A rectangle around a car tells us where the car appears in a picture. Locating it on Earth would also need camera geometry, aircraft position and attitude, gimbal pose, timing, and distance or terrain information.

## 2. The vocabulary, from scratch

### Programs, processes, and threads

A **program** is a set of instructions saved on disk. A **process** is a running instance of a program. A **thread** is a worker inside a process.

Think of a school workshop: the workshop is the process, and students at different benches are threads. They can work simultaneously but must coordinate access to shared supplies.

The camera's decoding thread can wait on video while ROS handles its own work. The detector's inference thread can process an image while the subscription callback receives another.

### ROS 2

**ROS 2** is a collection of tools and communication libraries for robotics software. Although its name expands to Robot Operating System, it runs on top of Linux here rather than replacing Linux.

ROS helps programs find each other, exchange typed messages, expose settings, and respond to requests.

| Term | Meaning | Example |
| --- | --- | --- |
| Package | Related source and build instructions | `zr10_ros2` |
| Executable | A compiled program you can start | `camera_stream_node` |
| Node | A participant in ROS communication | `/camera_stream` |
| Topic | Named channel carrying repeated messages | `/camera/image_raw` |
| Service | Request-and-response operation | `/take_photo` |
| Parameter | Named setting belonging to a node | `publish_fps` |

These are related but different. One executable can theoretically contain several nodes. These executables each create one main node.

### Topics are like radio channels

A **publisher** sends messages to a topic. A **subscriber** listens. Several programs can subscribe to the same images without the camera needing to know their internal code.

That separation is the point of the integration. A physical camera, a saved video, or another compatible publisher can provide image messages without changing the inference algorithm.

### Services are specific requests

A service accepts a request and sends a response. `std_srvs/srv/Trigger` has an empty request and a response containing `success` and `message`.

```bash
ros2 service call /take_photo std_srvs/srv/Trigger '{}'
```

This asks for one operation. It is different from listening to continuously arriving pictures.

### Parameters versus launch arguments

A node parameter is supplied like this:

```bash
ros2 run zr10_ros2 camera_stream_node --ros-args -p publish_fps:=5.0
```

A launch argument is supplied like this:

```bash
ros2 launch yolo26_ros camera_detection.launch.py publish_fps:=5.0
```

The launch file decides which arguments become which node parameters. It does not expose every parameter automatically.

Most project settings are read once at startup. Later changing ROS's stored parameter value does not necessarily update the C++ member actually controlling behavior. Restart with new startup values unless a dynamic update callback has explicitly been implemented.

### Images, models, and inference

A **pixel** is one colored element of a picture. A **frame** is one picture in a video. **FPS** is frames per second.

A 1920 × 1080 image has 2,073,600 pixels. A 30 FPS source produces about 30 pictures per second of source time. Source FPS, published FPS, and processed FPS can differ.

A neural-network **model** is a learned collection of numerical rules. **Training** adjusts those rules using examples. **Inference** applies them to a new image.

This runtime performs inference. It does not learn while watching video. Exporting a pretrained model during the build is not training either.

**ONNX** is a model representation that allows compatible software to load and execute a network. Here, C++ OpenCV loads the exported model.

## 3. The system map

The camera has different interfaces for different jobs:

```text
                          SIYI ZR10 camera
                       /         |          \
               UDP commands  HTTP photos  RTSP video
                    |            |             |
               zr10_node    syncPhotos()  camera_stream_node
                    |            |             |
           services/motion    JPEG files  /camera/image_raw
                                              |
                                      image_detector_node
                                         /          \
                       /detections/annotated_image   JPEG + CSV

Separately:
/gps/fix + /image_capture_time → gps_match_node → configured JPEG + JSON
```

Not every arrow is a ROS topic. UDP, HTTP, RTSP, files, and ROS messages are different ways to move information.

**Control:** a ROS service or motion message reaches `zr10_node`, which calls the binary UDP driver. This path does not carry video.

**Photo downloads:** an optional worker requests stored camera JPEGs through HTTP. It does not automatically capture photos on a repeating schedule.

**Live detection:** a stream reader decodes video and publishes images; the detector processes those messages. It does not need a `take_photo` call or camera recording to be enabled.

**Folder detection:** another executable detects new completed image files. This is an alternative input path, not the live subscriber's implementation.

**GPS:** the matching node receives GPS samples and capture-time events. The adapter connecting real images and their identities to this interface is still missing.

Separation lets us replace the camera with a file for testing, inspect each stage independently, and keep camera control in a process separate from expensive inference.

## 4. Repository tour

```text
ASTRA-Camera-System/
├── Dockerfile
├── compose.yaml
├── docker/entrypoint.sh
├── README.md
├── really_readme.md
├── data/                          # Host inputs and persistent results
└── src/
    ├── zr10_ros2/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/zr10.yaml
    │   ├── include/zr10_ros2/{zr10.h,media.h}
    │   └── src/
    │       ├── zr10.cpp
    │       ├── zr10_node.cpp
    │       ├── media.cpp
    │       └── camera_stream_node.cpp
    ├── yolo26_ros/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/yolo26_ros/
    │   ├── src/
    │   │   ├── yolo26_detector.cpp
    │   │   ├── image_detector_node.cpp
    │   │   ├── detector_node.cpp
    │   │   ├── file_watcher.cpp
    │   │   ├── video_feeder_node.cpp
    │   │   └── yolo26_cli.cpp
    │   ├── launch/
    │   └── tests/
    └── astra_geotagging/
        ├── include/astra_geotagging/
        └── src/
```

A header declares what a class or function offers. A `.cpp` file implements it: think of a restaurant menu versus the kitchen recipe.

`CMakeLists.txt` describes compilation and library linking. `package.xml` names the ROS package and dependencies. `colcon` discovers and coordinates package builds.

The detection source was imported from the neighboring `objectDetection` repository. This workspace contains its own copy. Editing the neighboring project does not automatically update this copy.

Some older camera documents describe a previous standalone controller, historical testing status, or features absent from the current ROS executable. Examples involving `./build/zr10`, periodic capture schedules, or a standalone FFmpeg relay are not the current `zr10_node` interface. Check current CMake targets and source when reconciling historical notes.

## 5. First experiment: video without a camera

### Prepare the environment

For the supplied Docker route, use a working Docker Engine and Compose plugin, a compatible Linux machine, enough disk space, Internet access for the first build, and a readable video file. The supplied image uses Ubuntu 22.04 and ROS 2 Humble.

The first build compiles OpenCV and exports a model, so it can take substantially longer than a camera-only build.

From the repository root:

```bash
docker compose build
mkdir -p data
```

Copy a real video into `data/test.mp4`. Its codec must be supported; an extension alone does not guarantee readable contents.

The host's `data/test.mp4` becomes `/data/test.mp4` inside the container through the Compose volume mount.

### Run the experiment

```bash
docker compose run --rm detection ros2 launch yolo26_ros camera_detection.launch.py \
  source:=/data/test.mp4 \
  output_dir:=/data/classroom-run-01 \
  tiling:=none
```

| Piece | Meaning |
| --- | --- |
| `docker compose run` | Start a one-off container |
| `--rm` | Remove that container after exit |
| `detection` | Use that service's image/network/mount configuration |
| `ros2 launch` | Execute a ROS launch description |
| `source:=...` | Use this file rather than the default RTSP address |
| `output_dir:=...` | Select a persistent run directory |
| `tiling:=none` | Use one full-image inference pass |

Explicitly targeting the service with `run detection` can start it without adding the profile option to this one-off command.

We choose `none` initially to simplify the experiment and reduce inference work. Later compare it with `auto`.

### Observe the result

The detector loads and warms up the model, logs `Ready: subscribing`, and the launch then starts the stream reader. The camera node logs `Camera source opened`.

Outputs appear in:

```text
data/classroom-run-01/
├── annotated_images/<seconds>_<nanoseconds>_<sequence>.jpg
└── csv/<seconds>_<nanoseconds>_<sequence>.csv
```

The file can finish while the ROS processes remain running. The stream worker returns at EOF, but its ROS node keeps spinning. Stop with **Ctrl-C**.

No recognized objects is still a valid result: a JPEG and a CSV containing only its header can be successful output.

### Run all services

```bash
docker compose --profile detection up
```

This starts default camera control, GPS matching, and optional detection. Camera control stays in dry-run under the supplied YAML. Detection still opens the real RTSP source, because streaming and control dry-run are separate.

To choose a file for that detection service:

```bash
CAMERA_SOURCE=/data/test.mp4 docker compose --profile detection up
```

## 6. Camera control

The main files are [zr10_node.cpp](src/zr10_ros2/src/zr10_node.cpp) and [zr10.cpp](src/zr10_ros2/src/zr10.cpp). The executable is `zr10_node`, and its default ROS node name is `zr10_camera`.

### The node is a translator

ROS users send services and typed messages. The camera expects a manufacturer-specific binary packet. For example:

```text
Trigger request → take_photo callback → ZR10::takePhoto()
                → command 0x0C, data 0x00 → UDP packet
```

With the default namespace, the services are:

| Service | Type | Intended operation |
| --- | --- | --- |
| `/take_photo` | `std_srvs/srv/Trigger` | Ask for a photo |
| `/autofocus` | `std_srvs/srv/Trigger` | Send the implemented autofocus command |
| `/toggle_recording` | `std_srvs/srv/Trigger` | Toggle recording |
| `/stop_zoom` | `std_srvs/srv/Trigger` | Send zero zoom motion |
| `/stop_gimbal` | `std_srvs/srv/Trigger` | Send zero yaw/pitch motion |
| `/center_gimbal` | `std_srvs/srv/Trigger` | Send center command |

A toggle flips state. Repeatedly calling it is not a reliable way to say “ensure recording is on,” especially when this code does not read the camera's state.

### Motion topics

`zoom_direction` uses `std_msgs/msg/Int8`: `1` means in, `-1` means out, and `0` means stop. Other values are rejected.

`gimbal_speed` uses `geometry_msgs/msg/Vector3`: `x` is yaw speed, `y` is pitch speed, and `z` is unused. Values must be finite and within `[-100, 100]`; accepted values are rounded to integer commands.

These are **speeds, not angles**. An `x` value of 20 does not mean “go to 20 degrees.” It is more like pushing a joystick in a direction.

### Dry run

`dry_run` defaults to true. Callbacks and internal motion state are exercised, but camera control packets are not sent. Responses say `DRY RUN: no camera command sent`. The driver still creates a UDP socket; creating it is different from transmitting commands.

Photo synchronization is disabled by the dry-run gate. The separate stream reader is not: select a local file if the experiment should avoid opening the physical camera stream.

### Motion watchdog

A timer checks motion every **50 ms**. If an active zoom or gimbal command has not been refreshed within `motion_timeout_sec`, default **0.5 seconds**, the node attempts to stop it.

A continuously moving command needs regular refreshes faster than that timeout, such as 10 Hz. Send zero motion explicitly when finished.

If the UDP stop send fails, the node retries during later checks and throttles repeated error logs. A successful local send still does not confirm a physical stop. During orderly destruction it also makes best-effort stop sends when live control is enabled. A dead process, broken network, or power failure can defeat a software watchdog.

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `camera_ip` | `192.168.144.25` | IPv4 control destination |
| `camera_port` | `37260` | UDP destination port |
| `dry_run` | `true` | Exercise callbacks without sending actions |
| `motion_timeout_sec` | `0.5` | Positive finite motion timeout |
| `sync_photos` | `false` | Enable photo worker when live mode also permits it |
| `media_base` | `http://192.168.144.25:82/cgi-bin/media.cgi` | Media API base |
| `photo_directory` | `photos` | Download destination |
| `sync_interval_sec` | `2.0` | Nominal scan-start spacing |
| `media_timeout_sec` | `15` | Per-transfer timeout |

Media parameters are declared inside the live synchronization startup branch. They may not appear as declared parameters when that branch is disabled. A YAML override is not proof a setting is currently used.

Changing `camera_ip` does not rewrite `media_base` or the stream's `source`. Update each relevant address independently.

### Binary packet layout

| Field | Size | Purpose |
| --- | --- | --- |
| Start | 2 bytes | `55 66` in hexadecimal |
| Control | 1 byte | Driver uses `01` |
| Data length | 2 bytes | Payload size, low byte first |
| Sequence | 2 bytes | Incrementing counter, low byte first |
| Command | 1 byte | Requested operation |
| Data | Variable | Operation arguments |
| CRC | 2 bytes | Error-detection value, low byte first |

`0x` indicates hexadecimal, a base-16 number. One byte holds 0–255. `0xFF` is the byte used for signed `-1` in the zoom-out payload.

The CRC calculation uses polynomial `0x1021`, a zero initial value, and a lookup table. It covers the preceding packet bytes. A CRC detects some corruptions; it is not encryption or acknowledgment.

| Operation | Command | Data bytes |
| --- | --- | --- |
| Photo | `0x0C` | `00` |
| Autofocus | `0x04` | `01` |
| Zoom in/out/stop | `0x05` | `01` / `FF` / `00` |
| Recording toggle | `0x0C` | `02` |
| Gimbal motion | `0x07` | Signed yaw byte, signed pitch byte |
| Center | `0x08` | `01` |

This table describes the current source. Firmware acceptance, especially autofocus protocol details, requires device verification.

The driver uses IPv4 UDP `sendto()` and checks the returned byte count. It has no camera acknowledgment receive loop. A successful live service means the **local UDP send succeeded**, not that a photo exists or the gimbal moved. Dropping a letter into a mailbox does not prove the recipient opened it.

## 7. Photo downloading

The implementation is [media.cpp](src/zr10_ros2/src/media.cpp). When both live control mode and `sync_photos` permit it, a worker:

1. Creates the destination directory.
2. Requests the camera's photo-directory inventory.
3. Gets a media count for each remote directory.
4. Requests photo listings in batches of 100.
5. Chooses a local path from each entry's complete URL.
6. Skips already existing targets.
7. Downloads unseen photos to temporary files.
8. Checks JPEG framing and renames successful downloads into place.

Endpoint suffixes appended to `media_base` include:

```text
/api/v1/getdirectories?media_type=0
/api/v1/getmediacount?media_type=0&path=<encoded-path>
/api/v1/getmedialist?media_type=0&path=<encoded-path>&start=<offset>&count=100
```

The worker expects JSON with `success`, `code`, and `data`. The assumed API needs validation against the actual firmware.

### URL handling and unusual filenames

Remote directory values are percent-encoded for URL queries so that spaces and reserved characters are sent as data.

For local storage, the full photo URL becomes hexadecimal text, split into nested directories when necessary. This avoids directly trusting arbitrary remote names and makes different complete URLs have distinct local identities.

The results can look strange rather than like friendly camera filenames. Logs retain the original remote name. Changed URLs can create new identities, and an existing target is skipped without a complete content revalidation. This is not a sophisticated remote-content database.

### Completion and validation

The download first goes to `.part`. After the transfer completes and framing passes, it is renamed to the final JPEG name. A reader is therefore less likely to consume a half-written final file.

The framing test requires the start marker `FF D8`. It scans backward past trailing zero padding and requires the final meaningful marker `FF D9`. It does not fully decode the JPEG and cannot guarantee every internal structure is correct.

Failed temporary downloads are removed and can be retried on later scans.

### Limits and scheduling

Listing-response bodies are limited to 8 MiB, individual downloads to 128 MiB, and returned photo counts to a nonnegative range up to one million. HTTP and HTTPS are allowed, but redirects are not followed. Connections have a three-second connection timeout; transfers use the configured timeout. A progress callback notices cancellation.

Scans never overlap. A long scan can exceed the nominal two-second interval, and the worker does not launch a burst of overdue scans afterward. Errors are logged while the outer loop continues.

### Why this does not directly feed the detector

Camera downloads can be nested. The folder detector watches only one directory and does not recurse. Setting its `watch_dir` to the photo root is therefore insufficient.

The implemented live connection uses image topics instead. A future full-resolution photo pipeline should explicitly report completed photo paths and their identities, or implement carefully designed recursive event handling.

## 8. Turning video into ROS images

The implementation is [camera_stream_node.cpp](src/zr10_ros2/src/camera_stream_node.cpp). Its executable is `camera_stream_node`; its ROS name is `camera_stream`.

### Reading compressed video

The default source is:

```text
rtsp://192.168.144.25:8554/video1
```

RTSP is a protocol used to request/control a network media stream. Compressed video must become actual pixels before inference.

The node uses OpenCV `VideoCapture` with the FFmpeg backend. It calls FFmpeg libraries through OpenCV, not a separate command-line `ffmpeg` process. Another stream path may be supplied if the device actually offers it; this code does not discover available streams automatically.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `source` | Default RTSP URL above | Nonempty RTSP URL or local video path |
| `frame_id` | `zr10_camera` | Coordinate-frame label in the header |
| `publish_fps` | `10.0` | Finite limit greater than 0 and at most 240 |
| `loop_video` | `false` | Reopen a local file after EOF |

A frame ID is a label. Naming it does not create a coordinate transform, provide calibration, or tell us the camera's orientation.

### Timeouts and reconnection

OpenCV receives an open-timeout request of 3000 ms and read-timeout request of 1000 ms. These are backend settings, not a universal hard deadline for every possible driver failure.

A failed open logs a warning, waits approximately three seconds, and retries. A failed live read breaks the current capture loop and returns to reconnection. The code recognizes live sources by a lowercase `rtsp://` or `rtsps://` prefix; it is not a universal network-source classifier.

The retry delay is split into short waits so shutdown can be noticed. No fake frame is substituted during a dropout.

### Decode rate and publication rate

A frame is decoded first. Then the node checks whether enough time has elapsed since the last publication. At 10 FPS the desired minimum interval is 100 ms.

Frames decoded too soon are not published. This reduces ROS traffic, but does not necessarily lower camera bitrate or eliminate the cost of decoding intervening frames.

Only `CV_8UC3` frames are published: three eight-bit channels. The ROS encoding is `bgr8`.

### Local video behavior

A file read without pacing could play a minute of video in a few seconds. This node instead paces local reads using the reported source FPS. If no usable FPS is reported, it falls back to the configured publication FPS.

The publication limit is checked separately. A 30 FPS file can be decoded at about 30 FPS while publishing at most about 10 FPS. The implementation is approximate FPS-based pacing, not a complete reconstruction of variable-frame-rate presentation timestamps.

A 5 FPS source does not become ten distinct source frames per second just because the publication limit is ten. This node does not intentionally duplicate frames. The older file feeder has different sampling behavior.

With looping disabled, EOF ends the worker but leaves the ROS process spinning. With looping enabled, the file is reopened after the outer retry delay, so there is an approximately three-second gap between loops.

### What the timestamp means

The node calls the ROS clock after decode:

```text
Exposure → compression → transfer → decode → assign header timestamp
   t0           t1           t2        t3              near t3
```

For accurate GPS matching, we would ideally know exposure time `t0` and the relationship between the camera and GPS clocks.

For a prerecorded file, the stamp is today's playback/decode time. It is neither the original recording date nor automatically the video's presentation timestamp.

## 9. The image message contract

An interface contract says exactly what one component promises and another expects. Here it is `sensor_msgs/msg/Image` with `bgr8` pixels.

| Field | Meaning |
| --- | --- |
| `header.stamp` | ROS-clock seconds and nanoseconds |
| `header.frame_id` | Camera coordinate-frame label |
| `height`, `width` | Pixel rows and columns |
| `encoding` | Byte interpretation; `bgr8` here |
| `is_bigendian` | Byte-order flag; emitted images set false |
| `step` | Bytes from the start of one row to the next |
| `data` | Image bytes |

### Pixel size and row stride

`bgr8` means blue, green, and red, each stored in eight bits. Eight bits make one byte, so each pixel needs three bytes.

For a tightly packed 640 × 480 image:

```text
step = 640 × 3 = 1,920 bytes
size = 1,920 × 480 = 921,600 bytes
```

Some images include padding after each row. Then `step` can exceed `width × 3`. The camera publisher packs rows tightly, but the detector correctly honors valid padded strides.

The current detector rejects `rgb8`, grayscale, RGBA, and other encodings. A different camera must publish `bgr8` or use a real conversion stage. Relabeling RGB bytes as BGR without swapping them changes the colors incorrectly.

### Validation

Before inference, the detector requires nonzero dimensions, width and height no greater than 32768, the exact encoding, stride at least `width × 3`, and data size at least `step × height`.

These checks avoid obvious malformed images. They do not establish meaningful content or an accurate timestamp.

### QoS and discovery

QoS means **Quality of Service**: communication policies for reliability, history, and durability. These image interfaces use sensor-data QoS with depth one, best-effort reliability, and no retained historical stream for later subscribers.

A reliable subscriber may not match a best-effort publisher. Seeing a discovered topic does not prove compatible message delivery. Diagnostic subscribers should use best effort for this pipeline.

ROS participants also need suitable network discovery and the same `ROS_DOMAIN_ID`. Think of a domain as choosing a particular school radio network: the same channel name in another network does not connect you.

### Raw images are large

A raw 1920 × 1080 BGR frame needs 6,220,800 bytes: about 6.22 MB or 5.93 MiB. At ten frames per second, pixel payload alone is about 62.2 MB/s before communication overhead. An annotated raw topic can add another similarly sized stream.

The compressed camera RTSP stream can be much smaller. Raw ROS image traffic and compressed camera traffic are different stages with different costs.

## 10. The live image detector

See [image_detector_node.cpp](src/yolo26_ros/src/image_detector_node.cpp). The executable is `image_detector_node`; its ROS name is `image_detector`.

### Startup

The constructor reads settings, validates them, creates output directories and an annotation publisher, and starts a worker. The worker loads and warms up the model. Only after readiness does the constructor create the image subscription and log `Ready: subscribing`.

Model construction and inference belong to one worker, avoiding concurrent calls on the same mutable OpenCV network.

| Parameter | Executable default | Meaning |
| --- | --- | --- |
| `model_path` | `/opt/model/yolo26.onnx` | ONNX model |
| `output_dir` | `annotated_output` | Saved result root |
| `conf_threshold` | `0.25` | Candidate score cutoff |
| `nms_threshold` | `0.45` | Suppression overlap threshold where applicable |
| `tiling` | `auto` | `auto`, `none`, or positive `WIDTHxHEIGHT` |

Thresholds must be finite and in `[0,1]`. The combined launch overrides the default output directory to `/data/detections`.

### One pending image

Suppose the worker is processing A. B arrives and waits. C arrives before A finishes and replaces B. After A, the worker processes C.

```text
Worker:       [ processing A ] [ processing C ]
Pending:            B → C           D → E
```

The callback only locks shared state, updates the pending message and replacement count, and wakes the worker. It does not perform inference.

A **mutex** is like the single key to a shared cupboard: only one worker can change the protected data at a time. A **condition variable** lets the worker sleep until work arrives rather than waste CPU repeatedly checking.

The application holds an in-flight frame and at most one pending frame, plus middleware and processing buffers. This bounds waiting work; it does not claim constant total memory regardless of image dimensions.

### Replacement logs

An illustrative line is:

```text
[1793300000_123456789_8] detections=2 detect=85.300ms replaced_pending=4
```

The frame produced two detections, its inference operation took 85.300 ms, and four waiting frames have been replaced since startup. This is a cumulative counter, not a number of missed objects or a count of all upstream drops.

### Processing steps

The worker takes the pending message, validates it, builds an OpenCV view, selects tiles, runs timed inference, draws on a clone, writes JPEG and CSV files, publishes an annotated message preserving the header, and logs completion.

Disk output comes before annotation publication. If writing fails, that frame may not reach the annotated topic despite successful inference.

Each output file is written atomically, but the two files are not one transaction. A successful JPEG followed by a CSV failure can leave an unmatched JPEG.

### Errors and stopping

A frame error logs `Frame failed` and the worker continues. A model failure is a startup failure before the subscription is ready.

Stopping wakes and joins the worker. An in-progress inference/write can finish first; pending work is not guaranteed to run. The implementation does not forcibly interrupt OpenCV halfway through inference.

## 11. Object detection mathematics

The shared library lives in [yolo26_detector.cpp](src/yolo26_ros/src/yolo26_detector.cpp), with its public interface in [yolo26_detector.hpp](src/yolo26_ros/include/yolo26_ros/yolo26_detector.hpp).

### Detection records and coordinates

A detection contains `x1, y1, x2, y2, confidence, classId`. Image coordinates begin at the top-left, with x increasing right and y increasing down.

A rectangle from `(100,50)` to `(220,190)` is roughly 120 pixels wide and 140 high. Those are pixels, not centimeters or GPS coordinates. The implementation quantizes through integer OpenCV rectangles; decimal formatting in CSV does not imply subpixel accuracy.

### Confidence and mistakes

Confidence is a model score, not a guaranteed calibrated probability. A score of 0.80 should not be presented as a universal promise of 80% correctness.

Raising the cutoff usually removes more weak guesses, potentially reducing false positives but increasing missed objects. A **false positive** reports something that is not there; a **false negative** misses something real.

### The label vocabulary

The code contains this ordered 80-class list:

```text
person, bicycle, car, motorcycle, airplane, bus, train, truck, boat,
traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat,
dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack,
umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball,
kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket,
bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple,
sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair,
couch, potted plant, bed, dining table, toilet, tv, laptop, mouse,
remote, keyboard, cell phone, microwave, oven, toaster, sink,
refrigerator, book, clock, vase, scissors, teddy bear, hair drier,
toothbrush
```

Class IDs select positions in that list. Out-of-range IDs become `unknown`. A custom model with different ordering needs a matching label map. Changing label text does not train recognition of a new object.

### Letterboxing

The network expects 640 × 640 pixels. Stretching a wide source into a square distorts its objects. The library preserves aspect ratio and pads unused space with gray value 114.

```text
scale = min(640 / source_width, 640 / source_height)
new_width = round(source_width × scale)
new_height = round(source_height × scale)
pad_x = floor((640 - new_width) / 2)
pad_y = floor((640 - new_height) / 2)
```

A 1920 × 1080 image becomes 640 × 360 with 140 pixels of padding above and below. This is **letterboxing**, like the bars around a widescreen movie.

### From image to tensor

`blobFromImage()` scales values by `1/255`, swaps BGR into RGB, and constructs the network input array. A channel value of 255 becomes about 1.0; 128 becomes about 0.502.

A **tensor** is a multidimensional numerical array. This one represents batches, color channels, rows, and columns.

The detector explicitly chooses `DNN_BACKEND_OPENCV` and `DNN_TARGET_CPU`. It runs a blank 640 × 640 warm-up image when constructed, before readiness is announced.

### Supported network outputs

| Export | Typical shape | Meaning |
| --- | --- | --- |
| End-to-end | `[1,300,6]` | Rows of corners, score, and class ID |
| Classic, 80 classes | `[1,84,8400]` | Four box channels plus class scores for many candidates |

The implementation expects a three-dimensional, 32-bit float tensor and uses the first output. It recognizes end-to-end output when the final dimension is six. Other graphs or semantic layouts can require decoder changes even if ONNX loading succeeds.

For classic output, it picks the strongest class score, converts center/width/height to corners, and suppresses overlapping boxes. End-to-end output is treated as already deduplicated within each tile; tile merging can still happen afterward.

### Mapping predictions back

Padding and scaling are reversed:

```text
source_x = (network_x - pad_x) / scale
source_y = (network_y - pad_y) / scale
```

Coordinates are clamped to the image. Tile offsets are added so final boxes refer to the whole source picture.

### Non-maximum suppression

Several boxes can describe one object. **NMS** keeps stronger boxes while suppressing sufficiently overlapping weaker ones.

Overlap is measured by intersection over union:

```text
IoU = overlap area / area covered by either rectangle
```

Identical rectangles have IoU 1, nonoverlapping ones 0. Lower overlap thresholds generally suppress more aggressively; higher thresholds let more overlapping boxes survive.

The current `NMSBoxes` calls do not split candidates by class. Suppression is class-agnostic, so differently labeled overlapping boxes can influence each other. That is an implementation detail worth understanding when evaluating crowded scenes.

## 12. Tiling

Imagine looking for a tiny bird in a large class photograph. Shrinking the whole picture may make it disappear. Looking at smaller pieces can preserve more useful detail per network input.

The choices are:

- `none`: one whole-image pass.
- `auto`: derive a grid from the actual image dimensions.
- `WIDTHxHEIGHT`: derive a fixed grid from those reference dimensions, then apply it to the actual image.

```text
columns = ceil(reference_width / 640)
rows = ceil(reference_height / 640)
```

| Reference | Columns | Rows | Passes |
| --- | --- | --- | --- |
| 640 × 480 | 1 | 1 | 1 |
| 1280 × 720 | 2 | 2 | 4 |
| 1920 × 1080 | 3 | 2 | 6 |
| 3840 × 2160 | 6 | 4 | 24 |

`tiling:=1280x720` does **not** resize the source, request a camera resolution, or set the saved output dimensions. It selects a 2 × 2 grid. On an actual 1920 × 1080 source, those four pieces are approximately 960 × 540 each. `auto` would instead use six approximately 640 × 540 pieces.

The partitions cover the image and do not intentionally overlap. Each tile is letterboxed independently, predicted boxes are shifted into global coordinates, and a final NMS merge is performed. Objects crossing tile boundaries can lose context; merging cannot restore information the model never saw.

More tiles cost additional inference. Six passes may be much slower than one, although the exact ratio is not guaranteed. Compare useful detections and mistakes, not just total box count: extra boxes can be false positives.

## 13. Understanding the outputs

### Annotated images

The library clones the input, draws green rectangles, and adds class labels and displayed scores. It does not draw directly into the original shared input.

Annotations keep source dimensions. The 640 × 640 network view is internal. The combined launch publishes annotations on `/detections/annotated_image`, preserving the input header rather than assigning a new completion stamp.

### Live filenames

```text
<stamp.sec>_<stamp.nanosec>_<sequence>.jpg
<stamp.sec>_<stamp.nanosec>_<sequence>.csv
```

The nanosecond number is ordinary decimal text, not necessarily padded to nine digits. The sequence increases as processing reaches output generation. The name does not contain `frame_id`.

Use separate directories for separate runs or detectors. Replaying identical stamped messages after a restart, or having two writers share a directory, can create collisions.

### CSV format

The exact header is:

```csv
x1,y1,x2,y2,confidence,class_id,class_name,resolutionX,resolutionY,detection_time
```

| Column | Interpretation |
| --- | --- |
| `x1,y1,x2,y2` | Box corners in original-image pixels |
| `confidence` | Model score |
| `class_id` | Numerical label index |
| `class_name` | Label from the fixed vocabulary |
| `resolutionX,resolutionY` | Original width and height |
| `detection_time` | Timed detector-library call, milliseconds |

Illustrative row:

```csv
100.00,50.00,220.00,190.00,0.8765,0,person,1920,1080,85.300
```

Each object in an image repeats that image's resolution and processing duration. Capture time is not a separate CSV column; the live filename associates the output with its stamped message. No detections produces a header-only CSV.

`detection_time` excludes drawing, JPEG/CSV writing, and output publication. Folder mode also excludes image reading and queue wait. A 50 ms inference measurement does not guarantee a complete result every 50 ms.

### Atomic writes

The shared helper writes a hidden temporary file and renames it to the final path after completion. Image temporary names retain an image extension for OpenCV's encoder. CSV uses a hidden temporary `.tmp` name.

This avoids exposing a partly written final file. It does not promise a two-file transaction, crash-proof disk persistence, or coordination between multiple writers choosing the same target.

## 14. The folder detector

[detector_node.cpp](src/yolo26_ros/src/detector_node.cpp) and [file_watcher.cpp](src/yolo26_ros/src/file_watcher.cpp) implement the older alternative input path. `detector_node` does not subscribe to image topics.

```text
video_feeder_node → completed files → FileWatcher → FIFO queue → inference
```

**FIFO** means first in, first out, like a lunch line. The queue stores filenames and arrival times rather than immediately keeping a pixel snapshot of every arrival.

### Watching completed files

Linux `inotify` reports events instead of requiring repeated directory scans. The watcher listens for `IN_CLOSE_WRITE`, meaning a writer closed a file, and `IN_MOVED_TO`, meaning a file was moved or renamed into the directory.

Writing a temporary file and renaming it into place is the stronger producer contract. A close-write event alone does not prove another process will never reopen that file.

The accepted extensions are `.png`, `.jpg`, `.jpeg`, `.bmp`, `.tif`, `.tiff`, and `.ppm`, compared case-insensitively. Hidden filenames and directory events are ignored.

It does not recurse into subdirectories or process preexisting files at startup. It tries image decoding up to four times, with short waits between retries. A corrupt image is logged as failed, and later items can continue.

### Parameters

| Parameter | Direct-executable default |
| --- | --- |
| `watch_dir` | `camera_feed_images` |
| `output_dir` | `annotated_output` |
| `model_path` | `yolo26.onnx` relative to the working directory |
| `conf_threshold` | `0.25` |
| `nms_threshold` | `0.45` |
| `tiling` | `auto` |

The test launch overrides the model path to `/opt/model/yolo26.onnx`. Pass that explicitly when running this executable directly in Docker.

The output root cannot equal or be inside the watched directory. Otherwise the detector could consume its own output and generate a feedback loop.

### Queue limits and repeated names

A set prevents adding the same name again while it is waiting. Its name leaves that set when dequeued, so a later event can queue it again. This is not a permanent processed-file database.

The queue is unbounded. If producers outpace inference, queue length and waiting time increase. Warnings appear as depth crosses increasing thresholds.

Because queued items refer to paths, overwriting an input before its turn can change what gets processed. FIFO events do not guarantee preservation of every historical version of each file. Keep inputs immutable when completeness matters.

An inotify overflow warning means arrivals may have been lost. There is no full-directory recovery scan. Losing or moving the watched directory is a watcher failure and triggers shutdown through a health timer.

### Output names and metrics

For input `img12.png`:

```text
annotated_images/annotated_img12.png
csv/img12.csv
```

Different extensions sharing one stem share the same CSV target. Repeated names can overwrite corresponding outputs.

| Log field | Meaning |
| --- | --- |
| `detect` | Timed inference operation |
| `queue_wait` | Event arrival to dequeue |
| `total` | Dequeue to output completion |
| `latency` | Event arrival to output completion |
| `queue_depth` | Remaining queue immediately after dequeue |

Shutdown prints processed, failed, and unprocessed counts, maximum depth, and mean/p95/max timings. The **mean** is the average, **p95** is a sorted-sample threshold covering about 95% of values, and **max** is the largest observed value.

Timing samples are retained in vectors for those summaries, so their storage grows over a long run even if the queue later empties.

### Shutdown is not a drain operation

Ctrl-C sets the stop flag. In-flight work can finish, but queued leftovers are not automatically processed. Let the producer finish and the backlog clear before stopping if every emitted sample is needed. The folder test launch deliberately keeps the detector alive after feeder EOF.

## 15. The original video feeder

`video_feeder_node` samples a video and writes images into a directory. It does not publish ROS image messages.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `video_path` | Empty; required | Local source video |
| `output_dir` | `camera_feed_images` | Image destination |
| `frame_interval_ms` | `100` | Sample spacing, 1–86,400,000 ms |
| `start_index` | `1` | First filename index, at least 1 |
| `file_prefix` | `img` | Nonhidden prefix without path separators |
| `image_format` | `png` | `png` or `jpg` |
| `png_compression` | `1` | PNG compression level, 0–9 |
| `loop` | `false` | Repeat the source |
| `dump_all` | `false` | Emit selected samples as fast as practical |

Desired source sample times are 0, 100, 200 ms, and so on at the default interval. The first normal timer callback occurs after an interval, but selects the source image at time zero.

The sampler decodes sequentially with a lookahead image. It chooses the latest source frame at or before each desired time. With a valid reported FPS, source timestamps come from decoded index and FPS. Otherwise it tries position timestamps and requires increasing usable times.

A 30 FPS source sampled every 100 ms produces about ten samples per source second. A 5 FPS source at the same sample spacing repeats some pictures, since new source frames arrive only every 200 ms.

`dump_all` changes emission speed, not the sample-time spacing. It does not mean “save every original source frame.”

Files are atomically written as `img1.png`, `img2.png`, etc. The directory is never cleared automatically. Restarting at index 1 can overwrite files; looping within one process keeps increasing output indices.

Normal EOF without looping shuts this feeder down. This differs from the newer camera stream node, whose ROS process keeps spinning after its decoding thread reaches EOF.

## 16. The standalone CLI

The CMake target `yolo26_cli` builds an executable named `yolo26_detect`. It does not create a ROS node, although `ros2 run` can locate and invoke it.

The model path is fixed to `yolo26.onnx` in the current working directory. A ROS `model_path` parameter does not configure this CLI.

Inside a suitable writable working directory, if that link does not already exist:

```bash
ln -s /opt/model/yolo26.onnx yolo26.onnx
ros2 run yolo26_ros yolo26_detect picture.jpg
```

Single-image syntax:

```text
yolo26_detect <image> [confidence] [nms] [--resolution WIDTHxHEIGHT]
```

It defaults to no tiling and writes `annotated.jpg` and `detections.csv` in the working directory.

Directory syntax:

```text
yolo26_detect <input_directory> <csv_directory> <annotated_directory> \
  [confidence] [nms] [--resolution WIDTHxHEIGHT]
```

It processes a sorted nonrecursive entry list, skips directories, and attempts decoding other entries. Directory mode defaults to **2 × 2 tiles**, not ROS `auto`. Output annotations use `<stem>_annotated<extension>`, which differs from the folder ROS node's prefix convention.

`--resolution` chooses a grid using the same ceiling formula; it does not change the saved source resolution.

The CLI uses `detectLegacy`, preserving historical merging behavior including explicit 1 × 1 tiling. Do not assume identical postprocessing for all superficially similar CLI and ROS configurations.

## 17. Launch files

A launch file describes which processes to start and how to configure them. It is Python orchestration, not the neural network itself.

### Camera detection launch

[camera_detection.launch.py](src/yolo26_ros/launch/camera_detection.launch.py) creates the stream reader and live detector. It remaps both `image_raw` interfaces to `/camera/image_raw`, and annotations to `/detections/annotated_image`.

| Argument | Default |
| --- | --- |
| `source` | `rtsp://192.168.144.25:8554/video1` |
| `publish_fps` | `10.0` |
| `loop_video` | `false` |
| `model_path` | `/opt/model/yolo26.onnx` |
| `output_dir` | `/data/detections` |
| `tiling` | `auto` |

`frame_id`, `conf_threshold`, and `nms_threshold` are node parameters but are not exposed by this launch. Run the nodes directly or extend the launch deliberately to change them.

The detector starts first. A process-output handler buffers stdout/stderr and looks for `Ready: subscribing`. It starts the camera once that text appears. This prevents a short video from playing during model loading.

This is readiness by log text, not a ROS lifecycle service. Changing that log requires updating the matcher. Detector exit triggers launch shutdown. There is no equivalent explicit camera-exit handler or automatic detector restart policy here.

### Folder test launch

[feed_test.launch.py](src/yolo26_ros/launch/feed_test.launch.py) starts the directory detector and waits for `Ready: watching` before starting the feeder.

Arguments are `video_path` (required), `model_path`, `watch_dir`, `output_dir`, `tiling`, `conf_threshold`, `nms_threshold`, `loop`, and `dump_all`.

It does not expose every feeder setting, such as `start_index` and PNG compression. The detector remains running after feeder EOF to process backlog. The launch does not automatically know when the queue has drained.

## 18. Docker and Compose

### Image versus container

An image is a prepared software environment. A container is a running instance. Docker reduces dependency differences across machines; it does not make hardware, architecture, camera firmware, or routing identical.

### The actual build stages

The current Dockerfile has three named stages: `opencv-build`, `model-export`, and `astra`. Some opening comments inherited from the original detection image describe a longer historical layout. The actual `FROM` instructions define what is built.

**OpenCV stage:** compiles OpenCV 4.11.0 into `/opt/opencv`, including `core`, `imgproc`, `imgcodecs`, `dnn`, and `videoio`. FFmpeg is enabled; GStreamer is disabled.

**Model stage:** uses Python 3.11, CPU PyTorch, Ultralytics, ONNX, and onnxslim. It downloads `yolo26n.pt`, exports ONNX, and names the result `yolo26.onnx`. Default export arguments are `END2END=True` and `OPSET=12`.

An **opset** versions the operations available to an ONNX graph. Compatibility depends on what the exporter produces and what the runtime supports.

**ASTRA stage:** starts from `ros:humble-ros-base`, installs dependencies, copies OpenCV and the model, and builds all three packages with `colcon`.

The model-export Python environment is not copied wholesale into runtime. Runtime inference is C++ OpenCV, not Python Ultralytics. The final image does retain source, compiler, and build tools; it is not a minimized runtime-only image.

### Smoke testing the model

The build creates a blank 640 × 480 PPM, links the model into a temporary working directory, and runs the CLI. Loading and inference must succeed for the image build to pass.

This establishes compatibility of that model/runtime combination. It does not establish accuracy on aircraft imagery or sufficient target-device speed.

### Version and reproducibility caveats

The Dockerfile declares `UBUNTU_VERSION`, `ROS_DISTRO`, `OPENCV_VERSION`, `END2END`, and `OPSET`. The final image and entrypoint explicitly use Humble, however, so changing `ROS_DISTRO` alone is not a migration to another ROS version.

Ultralytics and the Python export dependencies are not pinned to exact versions. Base tags and downloads can also change. A future rebuild is not guaranteed bit-for-bit identical.

For controlled results, record source revision, image identifier, model checksum, dependency versions, and export arguments. Stronger reproducibility requires additional pinning and artifact management.

An alternative export can be attempted with:

```bash
docker build --build-arg END2END=False -t astra-camera-system:humble .
```

The decoder has a classic-layout path, but every changed export still needs its own passing smoke test and evaluation.

### Why this OpenCV installation matters

The integrated packages request OpenCV 4.11 or compatible newer versions. A distribution package may provide an older release. Mixing a ROS bridge built against one OpenCV with another OpenCV in the same process can create binary compatibility problems.

This implementation builds message/image views manually rather than introducing `cv_bridge`. Both new nodes use the workspace's supplied OpenCV installation.

### Entrypoint

The entrypoint sources `/opt/ros/humble/setup.bash` and `/opt/astra/install/setup.bash`, then uses `exec` to start the requested process. The setup scripts teach the environment where ROS packages and libraries live. `exec` avoids leaving an unnecessary shell between the container and its application.

Compose also uses `init: true`, adding a small init process to help manage child processes.

### Services

| Service | Default role | Ordinary `up`? |
| --- | --- | --- |
| `camera` | Camera control with installed YAML | Yes |
| `geotagging` | GPS matching | Yes |
| `detection` | Stream reader plus image detector launch | Optional profile or explicit targeting |

All use the same image, host networking, selected domain ID, and `./data:/data` mount.

The camera service sets `/data/photos` but still disables photo sync by default. The GPS service supplies no input/output JPEG paths, so file writing is disabled by default. The service named `camera` runs control, while streaming is started inside the `detection` service.

### Files and ownership

The mount makes `/data` persistent in the host's `data` directory. Files elsewhere in a removed one-off container may disappear.

Containers run as root by default, so results may be root-owned. A non-root setup needs intentional writable data and ROS log directories, not just an arbitrary changed user ID. Rebuilding does not automatically delete persisted data or clear the camera SD card.

### Networking

Host networking shares the host's network environment rather than using a separate Docker NAT network. This helps ROS discovery and camera traffic, but does not create a route to the camera.

The host still needs a working physical connection, interface configuration, and route to the camera subnet. Firewalls and ROS discovery settings still matter. Linux Docker Engine is the intended setup; Docker Desktop can need host-networking support enabled.

For a camera at `192.168.144.25`, the camera-facing interface needs suitable reachability to its subnet. Inspect the existing route before changing network settings; blindly replacing the default route can disconnect unrelated networking.

## 19. Native builds

Native means running directly in an installed ROS environment. The project uses C++17 and needs ROS tools/messages, libcurl, nlohmann JSON headers, Exiv2, pkg-config, and the required OpenCV modules with FFmpeg.

With an already prepared Humble environment and OpenCV under `/opt/opencv`:

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4
source install/setup.bash
```

`OpenCV_DIR` is the directory containing the CMake package configuration, not the source or include directory. Change it to match your real installation.

Runtime libraries must also be discoverable. One shell-level example is:

```bash
export LD_LIBRARY_PATH=/opt/opencv/lib:${LD_LIBRARY_PATH:-}
```

The supplied image already sets its own library path. A native machine can alternatively manage loader configuration system-wide.

`BUILD_TESTING=OFF` matches Docker and avoids requiring optional geotagging lint dependencies. The custom integration checks are separate from a blanket `colcon test` command.

Each new native terminal needs both ROS and workspace setup files sourced. Native execution also needs a compatible ONNX file:

```bash
ros2 launch yolo26_ros camera_detection.launch.py \
  source:=/absolute/path/test.mp4 \
  model_path:=/absolute/path/yolo26.onnx \
  output_dir:=/tmp/astra-native-run \
  tiling:=none
```

Humble Docker is the tested environment. Different ROS versions, operating systems, Jetson images, or CPU architectures need their own build and runtime validation.

## 20. GPS and the missing connections

Geotagging attaches geographic information to an image. The package contains a logging subscriber, a matching node, a pure matching function, and a JPEG metadata writer.

`gps_subscriber` observes `/gps/fix` and rejects the explicit no-fix status. `gps_match_node` validates and buffers samples, receives capture events, matches them, and optionally writes output.

```text
/gps/fix              sensor_msgs/msg/NavSatFix
/image_capture_time   std_msgs/msg/Int64
```

Capture times are Unix milliseconds. GPS seconds/nanoseconds are converted to milliseconds. They must share the same clock basis.

### Nearest-time matching

The node retains the last 100 valid received samples. For each capture event, it picks the smallest absolute time difference and allows at most **200 ms**.

```text
Capture: 10,000 ms
GPS A:    9,900 ms → difference 100 ms
GPS B:   10,040 ms → difference  40 ms
GPS C:   10,250 ms → difference 250 ms
```

B is the nearest acceptable candidate among those values.

Matching happens immediately with samples already buffered. The node does not wait for future samples, revisit old matches, or interpolate positions. A 100-sample buffer covers roughly ten seconds at 10 Hz or 100 seconds at 1 Hz, assuming regular input.

| Status | Meaning |
| --- | --- |
| `Matched` | Nearest usable sample is close enough |
| `InvalidCaptureTime` | Capture time is not valid |
| `NoValidGps` | No suitable sample exists |
| `OutsideTolerance` | Closest usable sample is too far away in time |

The node checks fix status, finite coordinates, positive timestamps, and coordinate ranges. It assigns internal fix type 3 as a simulation placeholder, not a decoded MAVLink fix report.

### Metadata output

Parameters `input_jpeg` and `output_dir` default empty. Without both, file writing is disabled.

A match causes a copy of the same configured input JPEG to be written with a timestamp/counter name. Exiv2 adds GPS EXIF fields; a JSON sidecar includes capture/GPS times, latitude, longitude, `altitude_msl_m`, fix type, difference, source, and match status.

The JSON hard-codes `gps_source` to `simulated`. The internal altitude name also needs care: incoming `NavSatFix.altitude` follows an ellipsoidal convention, while the code copies it directly into a field named `altitude_msl_m` without geoid/MSL conversion. Renaming a value does not change its physical datum.

### The missing image adapter

The streaming camera publishes `Image`, while the matcher expects an `Int64` capture event and uses a fixed input file. No current adapter selects a real image, identifies its saved path, establishes exposure time, requests a GPS association, and joins the metadata to its detection CSV.

A future event should carry image identity and time together. It should establish clocks and decide whether to wait for later GPS. Publishing an unrelated fresh wall-clock time does not solve image association.

Geotagging a camera image also does not locate each visible object on the ground. That requires calibration, camera/gimbal pose, aircraft attitude, altitude/range, and terrain or another scene model.

## 21. Command cookbook

Docker commands here run on the host from the repository root. Commands in an explicitly opened container run inside that container. Supply your own readable video/image files.

### Inspect configuration before running

```bash
docker compose --profile detection config
```

This renders environment substitutions without starting services.

### Default services and optional detection

```bash
docker compose build
docker compose up
```

The ordinary setup starts dry-run camera control and GPS matching, without optional detection. To include detection:

```bash
docker compose --profile detection up
```

To start only its long-running service:

```bash
docker compose --profile detection up detection
```

To stop the service containers later:

```bash
docker compose down
```

### Replay a file

```bash
docker compose run --rm detection ros2 launch yolo26_ros camera_detection.launch.py \
  source:=/data/test.mp4 \
  loop_video:=true \
  publish_fps:=5.0 \
  tiling:=none \
  output_dir:=/data/replay-run-01
```

Loops include the reopen gap. Frames receive new playback/decode timestamps.

### Read logs

```bash
docker compose logs -f detection
```

A one-off `compose run` prints its own logs in its attached terminal; a separately running one-off container is not the same as the normal service container targeted by `compose exec`.

### Inspect a running service

While the normal detection service is running:

```bash
docker compose exec detection bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/astra/install/setup.bash && ros2 node list'
```

`exec` runs another command inside an existing container. It does not rerun the entrypoint, so this command sources the setup explicitly.

Alternatively, open a one-off prepared shell:

```bash
docker compose run --rm detection bash
```

Inside that shell:

```bash
ros2 pkg executables zr10_ros2
ros2 pkg executables yolo26_ros
ros2 node list
ros2 topic list -t
ros2 topic info /camera/image_raw --verbose
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --field header --qos-reliability best_effort
```

`hz` measures what its observer receives, not sensor exposure rate or detector throughput. Header-only echo avoids printing millions of pixel bytes.

### Dry-run service request

With the default camera service running:

```bash
docker compose run --rm camera ros2 service call /take_photo std_srvs/srv/Trigger '{}'
```

The response should identify the dry run. It does not create a local photo.

### Enable live control and downloads

Once an appropriate physical test setup is arranged, stop the default controller to avoid concurrent controllers:

```bash
docker compose stop camera
```

Then run the explicit live configuration:

```bash
docker compose run --rm camera ros2 run zr10_ros2 zr10_node --ros-args \
  --params-file /opt/astra/install/zr10_ros2/share/zr10_ros2/config/zr10.yaml \
  -p dry_run:=false \
  -p sync_photos:=true \
  -p photo_directory:=/data/photos
```

This allows real sends and photo downloads, not periodic automatic captures. Use a separate photo service request when appropriate. A software watchdog and a successful UDP response do not establish physical completion or substitute for a physical stop method.

### Motion messages

Inside a prepared ROS shell, these send example commands to the current controller:

```bash
ros2 topic pub --once /zoom_direction std_msgs/msg/Int8 '{data: 1}'
ros2 topic pub --once /zoom_direction std_msgs/msg/Int8 '{data: 0}'
ros2 topic pub --once /gimbal_speed geometry_msgs/msg/Vector3 '{x: 10.0, y: 0.0, z: 0.0}'
ros2 topic pub --once /gimbal_speed geometry_msgs/msg/Vector3 '{x: 0.0, y: 0.0, z: 0.0}'
```

They exercise dry-run behavior under the default configuration and send real motion requests if a live controller is running. A single nonzero message is subject to the motion watchdog timeout.

### Stream publication alone

```bash
docker compose run --rm detection ros2 run zr10_ros2 camera_stream_node --ros-args \
  -p source:=/data/test.mp4 \
  -p publish_fps:=5.0 \
  -r image_raw:=/camera/image_raw
```

No detector is started. Inspect publication separately.

### Detector alone with different thresholds

Start this first, then a compatible publisher in another terminal:

```bash
docker compose run --rm detection ros2 run yolo26_ros image_detector_node --ros-args \
  -p model_path:=/opt/model/yolo26.onnx \
  -p output_dir:=/data/threshold-run-01 \
  -p conf_threshold:=0.50 \
  -p nms_threshold:=0.45 \
  -p tiling:=none \
  -r image_raw:=/camera/image_raw \
  -r annotated_image:=/detections/annotated_image
```

`-r` remaps source-code names onto shared topic names. An unrelated publisher still needs `bgr8`, valid data, and compatible QoS; compressed-image messages are not accepted directly.

### Folder test pipeline

```bash
docker compose run --rm detection ros2 launch yolo26_ros feed_test.launch.py \
  video_path:=/data/test.mp4 \
  watch_dir:=/data/folder-run-01/input \
  output_dir:=/data/folder-run-01/output \
  model_path:=/opt/model/yolo26.onnx \
  tiling:=none
```

Use fresh directories. Allow the backlog to finish after feeder EOF before stopping if all samples are required.

### One-image CLI

With `data/photo.jpg` available and a fresh run directory:

```bash
docker compose run --rm detection bash -lc \
  'mkdir -p /data/cli-run-01 && cd /data/cli-run-01 && ln -s /opt/model/yolo26.onnx yolo26.onnx && ros2 run yolo26_ros yolo26_detect /data/photo.jpg'
```

Results appear in `data/cli-run-01`. The explicit link satisfies the CLI's working-directory model lookup. It intentionally does not replace an existing link/model.

### GPS file-writing experiment

With `data/input.jpg` available:

```bash
docker compose stop geotagging
docker compose run --rm geotagging ros2 run astra_geotagging gps_match_node --ros-args \
  -p input_jpeg:=/data/input.jpg \
  -p output_dir:=/data/geotagged
```

It still requires valid GPS and capture events on matching clocks. Selecting a JPEG does not connect it to the stream or trigger a match by itself.

## 22. Performance and tradeoffs

### Three rates can coexist

Hypothetically:

```text
Camera produces:       30 FPS
ROS publisher sends:   10 FPS
Detector completes:     4 FPS
```

There is no contradiction. The publisher selects frames, and the detector cannot finish all published input. The live pending slot replaces waiting images; the folder queue grows.

If files arrive at ten per second while processing handles four, backlog grows by about six per second. After a minute, about 360 files wait. With no new arrivals, four per second would take about 90 seconds to clear them. These are illustrative values, not measurements of this model.

### Throughput versus latency

**Throughput** is completed work per second. **Latency** is time between defined stages for an individual item. A system can maintain reasonable throughput while displaying old results because a long queue sits in front of its worker.

Latest-pending replacement favors freshness over completeness. A managed file queue favors eventually processing stored inputs but must account for overload, storage, event loss, restarts, and immutable files.

The live `detect` measurement is not camera-to-result latency. Even output time minus the header stamp omits exposure-to-decode delay because the stamp is assigned after decoding.

### Where time and memory go

Costs include decoding, pixel copies, transport, preprocessing, each inference pass, merging, drawing, JPEG encoding, disk writes, and work done by other subscribers.

Lowering publication FPS reduces message opportunities. Fewer tiles reduce network passes. A lower-resolution camera stream can reduce decode/transport work if the device provides it. None guarantees improved accuracy.

Output files accumulate without a built-in retention period or quota. The live waiting policy is bounded, but disk usage is not automatically bounded. The folder detector also retains timing samples for exact summaries.

### GPU expectations

This library explicitly chooses CPU inference. The supplied Docker configuration does not create a CUDA/TensorRT inference path. A GPU port needs compatible libraries, backend/model choices, and correctness and performance checks; changing one enum is not a complete port.

### A useful tuning experiment

Use one representative video, one machine, fresh output directories, and one changed variable at a time. Record source size, publication rate, model identity, thresholds, tiling, useful true detections, misses, false alarms, timings, queue/replacement behavior, and storage use.

More output boxes are not automatically better results. Keep accuracy and resource use in the same discussion.

## 23. Testing

Sources live under [src/yolo26_ros/tests](src/yolo26_ros/tests); recorded checks are in [VALIDATION.md](src/yolo26_ros/tests/VALIDATION.md).

### Model smoke test

The Docker build requires the exported model to load and infer on a blank image. It establishes runtime compatibility, not accuracy.

### Topic integration

```bash
docker compose run --rm -v "$PWD/src/yolo26_ros/tests:/checks:ro" detection \
  python3 /checks/topic_integration.py
```

This starts the detector, sends a malformed message, then sends valid padded-row image data. It checks real inference, error recovery, JPEG/CSV output, image dimensions/encoding, and timestamp/frame-ID preservation.

It uses `/test/...` topics. Multiple concurrent test sessions should use separate `ROS_DOMAIN_ID` values to avoid interfering.

### Stream integration

Provide your own readable `data/test.mp4`:

```bash
docker compose run --rm -v "$PWD/src/yolo26_ros/tests:/checks:ro" detection \
  python3 /checks/stream_integration.py /data/test.mp4
```

This checks real local decoding/publication, basic headers, byte sizes, configured rate spacing, and stopping the process. It is not a physical RTSP dropout test.

### Combined launch

```bash
docker compose run --rm -v "$PWD/src/yolo26_ros/tests:/checks:ro" detection \
  python3 /checks/launch_integration.py /data/test.mp4
```

This verifies detector readiness before camera startup, video-to-topic-to-inference output, and launch shutdown.

### Core C++ checks

`core_checks.cpp` checks layout math, malformed resolutions, path containment, CSV, annotation, atomic writing, and inotify without an ONNX model or ROS node. It is not automatically registered in CMake as a `colcon test` target.

The supplied image retains a compiler and headers. To build and run it explicitly:

```bash
docker compose run --rm detection bash -lc \
  'g++ -std=c++17 -UNDEBUG /opt/astra/src/yolo26_ros/tests/core_checks.cpp /opt/astra/src/yolo26_ros/src/yolo26_detector.cpp /opt/astra/src/yolo26_ros/src/file_watcher.cpp -I/opt/astra/src/yolo26_ros/include -I/opt/opencv/include/opencv4 -L/opt/opencv/lib -Wl,-rpath,/opt/opencv/lib -lopencv_core -lopencv_dnn -lopencv_imgproc -lopencv_imgcodecs -pthread -o /tmp/astra-core-checks && /tmp/astra-core-checks'
```

`-UNDEBUG` keeps its assertions active. `video_sampling_checks.cpp` separately compares original feeder samples against expected source pixels for suitable constant-frame-rate inputs and the default interval. It is not a test of the newer wall-clock-paced stream node.

### The boundary of software evidence

These tests do not prove firmware command acceptance, optical focus, SD-card performance, all network reconnection cases, real-image accuracy, target-device throughput, exposure timestamps, or GPS synchronization. Those need representative hardware and data experiments.

## 24. Troubleshooting

The most useful question is: **where did expected data stop appearing?** Follow the pipeline rather than changing random settings.

### Package or executable not found

Check that the terminal is in the prepared container or has sourced the correct native workspace. Use `ros2 pkg executables zr10_ros2` and `ros2 pkg executables yolo26_ros`.

Host edits do not hot-reload installed container executables: source is copied at build time. Rebuild an old image when executables change.

### OpenCV build failure

Inspect the selected version and `OpenCV_DIR`. A system OpenCV 4.5 does not satisfy the 4.11 requirement. Finding headers at compile time does not prove runtime libraries resolve to the same installation; use `ldd` when checking shared-library resolution.

### Build is slow or exhausts memory

First builds compile OpenCV and download model/export dependencies. Later builds can reuse cache. OpenCV uses `nproc` parallelism; too many simultaneous compiler processes can exhaust a small machine's memory. Reduce build parallelism if logs identify that problem. Read which stage actually failed before changing camera settings.

### Cannot open source

For a file, check the path inside the container. A host home-directory path is not visible without a mount; use `/data/...` for the standard volume.

For RTSP, check physical connection, route, address, port, stream path, and whether an independent known client can open it. Ping alone is not conclusive because some devices do not answer it.

Disabling control dry-run does not repair an unreachable stream.

### Camera opens but no results appear

Check readiness, camera-open logs, topic publisher/subscriber counts, message types and QoS, domain IDs, frame/write errors, ongoing tiled inference, and the actual mounted output directory.

A short video started independently before the detector is ready can finish too early. Use the combined launch or start the subscriber first.

### Invalid BGR message

Check encoding, dimensions, stride, and byte count. Convert pixels properly; do not simply relabel RGB as BGR.

### Topic exists but observer is silent

Compare QoS using verbose topic information. A reliable observer may not match a best-effort publisher. Check domain IDs and discovery/network configuration. Topic discovery and successful data delivery are different facts.

### No boxes but valid files

A header-only CSV is a valid zero-detection result. Use scenes containing supported classes, and consider blur, lighting, object scale, thresholds, tiling, and model suitability. Blank images are useful compatibility tests, not recognition demonstrations.

### Wrong labels or boxes with a new model

Verify preprocessing, tensor layout, coordinate semantics, and class ordering. Compatible ONNX syntax is not enough to establish a compatible decoder or label vocabulary.

### Replacement count rises

The detector receives faster than it processes. That is its intended live policy. Measure, compare fewer tiles or a lower publication rate, and decide whether the task values freshness or completeness.

### Folder inputs ignored or delayed

Were they preexisting, hidden, nested, or an unsupported extension? Did the detector start before completion events? Was the same path overwritten while waiting?

A growing queue increases delay. Its logs distinguish queue wait from inference. Changing the stored parameter after startup does not reconstruct the watcher; restart with the intended path.

### Service success without camera action

Check whether the response says dry run. If live, success still only means local UDP send. Verify network, firmware protocol, payload, and actual device state. This driver does not receive acknowledgments to resolve those questions.

### No downloaded photos

Both live mode and sync must permit the worker. The camera needs JPEGs to list, the HTTP API must match firmware, and the destination must be writable.

A sent capture request is not proof of SD storage, HTTP listing, successful transfer, or local JPEG creation. Those are separate stages.

### Missing files or permission denied

Inspect the mounted path and ownership. `/tmp` inside a removed one-off container is not persistent `/data`. Choose an intentional writable directory and ownership policy; a broad recursive permission change is not a diagnosis.

### GPS never matches

Check valid fixes, positive GPS timestamps, capture milliseconds rather than seconds, matching clock basis, and samples arriving before capture events. Seconds versus milliseconds creates a factor-of-1000 error. Simulated ROS time versus unrelated Unix wall time can also prevent matching.

A match still does not write files unless both path parameters are set.

## 25. Extending the project

### Another camera publisher

Match `sensor_msgs/Image`, `bgr8`, valid stride/data, and QoS, then remap the detector input. Compressed or differently encoded images need a tested conversion stage.

### Structured detections

A useful output could carry the original header, classes, scores, and boxes in a standard or intentionally designed ROS type. Specify pixels versus normalized values, corner conventions, and zero-detection behavior. Preserve image identity.

### Full-resolution photo processing

Add a download-completed event with a stable identity and exact path. Decide queue completeness, restart behavior, and file immutability. Recursive watching is another option, but must handle newly created directories, startup files, completion, and overflow correctly.

### Trustworthy GPS association

Trace time from exposure, establish clock synchronization, carry identity and timestamp together, and decide whether to wait for later GPS. Consider uncertainty. Perfectly preserving an inaccurate timestamp does not make it accurate.

### Custom models

Collect representative labeled data, train and evaluate outside runtime, export a compatible graph, and update labels/decoder as needed. Keep evaluation data separate from training data. Aircraft viewpoints and tiny objects can differ greatly from familiar street photography.

### Tracking and automatic gimbal motion

Tracking needs persistent identities. Two `person` detections in different images do not establish that they are the same person.

Automatic control additionally needs feedback, sign conventions, rate limits, a policy, and physical validation. A pixel offset is not automatically a safe yaw angle or speed command.

### Testable changes

Change clear interfaces deliberately, test pure mathematics independently, and use integration tests for cross-process behavior. Keep implemented features, observed results, and future plans distinguishable in documentation.

## 26. Classroom exercises

### Follow one picture

Draw the path from saved video to annotated JPEG. Mark compressed-to-pixel conversion, topic publication, pending slot, inference, and disk output.

Expected path: file → OpenCV/FFmpeg → stream message → detector → inference library → annotation/CSV.

### Account for missing frames

With a 30 FPS source, 10 FPS publication, and four processed FPS, name three places frames can disappear.

Expected answer: publisher rate selection, best-effort transport, and pending-frame replacement. These are different mechanisms.

### Calculate image storage

A tightly packed 1280 × 720 BGR image needs:

```text
1280 × 720 × 3 = 2,764,800 bytes
```

With row padding, use `step × height` instead. Why is width alone insufficient? Because stride may include unused bytes at row ends.

### Predict tiles

For a 1920 × 1080 actual image: `auto` chooses 3 × 2; `none` one whole-image pass; `1280x720` a fixed 2 × 2 grid on the actual source.

Explain why the fixed reference does not resize the output.

### Choose a queue policy

Would you replace waiting frames for a live operator display? Would you do that when every recorded photo must be analyzed? Explain freshness versus completeness and what overload handling is needed.

### Interpret zero detections

A CSV has only a correct header and a JPEG exists. Is this necessarily failure? No: the frame may have processed successfully with no candidates meeting the criteria.

### Challenge a success message

What does a successful live `take_photo` service establish? Local UDP transmission. Camera receipt, execution, SD storage, HTTP listing, and local download remain separate questions.

### Estimate timing error

An illustrative aircraft moving at 10 m/s travels this far in 200 ms:

```text
10 m/s × 0.2 s = 2 m
```

This is motion during an offset, not a complete geolocation error estimate. Geometry, GPS uncertainty, attitude, and exposure/transport delays add other effects.

### Run a controlled comparison

Use one video and three fresh directories with `none`, `auto`, and `1280x720`. Compare useful detections, mistakes, inference times, replacement counts, and disk use. State what you held constant and what your results cannot establish.

## 27. Reference and glossary

### Executables and default node names

| Package | Executable | ROS node |
| --- | --- | --- |
| `zr10_ros2` | `zr10_node` | `zr10_camera` |
| `zr10_ros2` | `camera_stream_node` | `camera_stream` |
| `yolo26_ros` | `image_detector_node` | `image_detector` |
| `yolo26_ros` | `detector_node` | `detector_node` |
| `yolo26_ros` | `video_feeder_node` | `video_feeder_node` |
| `yolo26_ros` | `yolo26_detect` | None |
| `astra_geotagging` | `gps_match_node` | `astra_gps_matcher` |
| `astra_geotagging` | `gps_subscriber` | `astra_gps_subscriber` |

Names can be remapped; the table describes defaults.

### Main interfaces

| Interface | Type | Producer → consumer |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Stream → detector under combined launch |
| `/detections/annotated_image` | `sensor_msgs/msg/Image` | Detector → compatible observer |
| `/zoom_direction` | `std_msgs/msg/Int8` | External commands → camera control |
| `/gimbal_speed` | `geometry_msgs/msg/Vector3` | External commands → camera control |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | External GPS → GPS nodes |
| `/image_capture_time` | `std_msgs/msg/Int64` | External capture events → matcher |

The default project does not supply the external command, GPS, or capture-event publishers in this table.

### Container paths

| Path | Purpose |
| --- | --- |
| `/opt/astra/src` | Source copied into image |
| `/opt/astra/install` | Installed workspace |
| `/opt/opencv` | Custom OpenCV |
| `/opt/model/yolo26.onnx` | Exported model |
| `/data` | Host data mount under Compose |
| `/data/detections` | Default combined-launch outputs |
| `/data/photos` | Compose control service's photo destination |

### Glossary

| Term | Meaning |
| --- | --- |
| Annotation | Visible box or label added to an image |
| API | Agreed software interface |
| Atomic rename | Making a completed file visible at its final name in one rename operation |
| Backend | Implementation behind an interface, such as FFmpeg |
| BGR | Blue, green, red channel order |
| Bounding box | Rectangle identifying an image region |
| Calibration | Measuring camera properties needed for physical geometry |
| Callback | Function invoked when an event occurs |
| Class | Recognized object category |
| CLI | Command-line interface |
| Codec | Compression/decompression method |
| CRC | Error-detection value calculated from bytes |
| DDS | Communication technology used by common ROS 2 middleware |
| Decode | Convert compressed media into pixels or samples |
| EXIF | Metadata stored in an image |
| FIFO | First in, first out |
| Frame ID | Coordinate-frame label in a message |
| Inference | Applying a trained model |
| Inotify | Linux filesystem-event notification |
| IoU | Intersection over union: box overlap measure |
| JPEG | Commonly lossy image compression format |
| Latency | Time between defined stages for an item |
| Letterbox | Resize preserving aspect ratio, then pad |
| Mutex | Lock protecting shared data |
| NMS | Suppression of weaker overlapping predictions |
| ONNX | Portable model representation |
| Pixel | One colored element of a digital image |
| PNG | Lossless image format used by the original feeder |
| QoS | Reliability/history/durability policies |
| Remapping | Connecting source-code names to chosen ROS names |
| RTSP | Network media stream protocol |
| Step/stride | Byte distance between image rows |
| Tensor | Multidimensional numerical array |
| Throughput | Work completed per unit time |
| Timestamp | Numerical point in time |
| UDP | Datagram transport without application-execution confirmation |
| Watchdog | Periodic check reacting when updates stop |

### Suggested source-reading order

1. [camera_stream_node.cpp](src/zr10_ros2/src/camera_stream_node.cpp): follow a decoded frame into a message.
2. [image_detector_node.cpp](src/yolo26_ros/src/image_detector_node.cpp): follow a message through pending work and output.
3. [yolo26_detector.hpp](src/yolo26_ros/include/yolo26_ros/yolo26_detector.hpp): learn the public inference interface.
4. [yolo26_detector.cpp](src/yolo26_ros/src/yolo26_detector.cpp): inspect preprocessing, decoding, tiles, NMS, CSV, and drawing.
5. [camera_detection.launch.py](src/yolo26_ros/launch/camera_detection.launch.py): inspect remappings and startup order.
6. [zr10_node.cpp](src/zr10_ros2/src/zr10_node.cpp): services, motion callbacks, and watchdog.
7. [zr10.cpp](src/zr10_ros2/src/zr10.cpp): packet assembly and UDP sends.
8. [media.cpp](src/zr10_ros2/src/media.cpp): HTTP inventory and completed-photo handling.
9. [detector_node.cpp](src/yolo26_ros/src/detector_node.cpp) and [file_watcher.cpp](src/yolo26_ros/src/file_watcher.cpp): compare folder queues with live messages.
10. [gps_match_node.cpp](src/astra_geotagging/src/gps_match_node.cpp): identify what a future image/GPS adapter must provide.
