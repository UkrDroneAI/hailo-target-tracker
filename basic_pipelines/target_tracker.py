from hailo_rpi_common import (
    get_default_parser,
    QUEUE,
    get_caps_from_pad,
    get_numpy_from_buffer,
    GStreamerApp,
    app_callback_class,
)
from pymavlink_common import wait_heartbeat, auto_connect
import supervision as sv
import hailo
import time
import cv2
import setproctitle
import numpy as np
import multiprocessing
import argparse
import os
from gi.repository import Gst, GLib
import gi

from autopilot_companion import AutipilotCompanion
gi.require_version('Gst', '1.0')


ac = AutipilotCompanion()

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
# Inheritance from the app_callback_class


class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42  # New variable example

    def new_function(self):  # New function example
        return "The meaning of life is: "

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------

# This is the callback function that will be called when data is available from the pipeline


def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    # Check if the buffer is valid
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    hailo_detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    n = len(hailo_detections)

    # Get the caps from the pad
    _, w, h = get_caps_from_pad(pad)

    boxes_dict = {}
    confidence_dict = {}
    class_id_dict = {}

    for i, detection in enumerate(hailo_detections):

        tracker_id = detection.get_objects_typed(
            hailo.HAILO_UNIQUE_ID)[0].get_id()
        
        class_id_dict[tracker_id] = detection.get_class_id()
        confidence_dict[tracker_id] = detection.get_confidence()
        bbox = detection.get_bbox()
        boxes_dict[tracker_id] = [
            bbox.xmin(), bbox.ymin(), bbox.xmax(), bbox.ymax()]

    
    ac.process_video_frame(
        class_ids=class_id_dict,
        confidences=confidence_dict,
        boxes=boxes_dict,
        video_frame_size=(w,h)
    )
    
    # if confidence.size != 0:
    #     most_confident_target_id = confidence.argmax()
    #     if ac.target_id_locked not in tracker_id:
    #         print("The target has been lost")
    #         # TODO implement lost status processing
    #         ac.lock_target(tracker_id[most_confident_target_id])
    #         print(
    #             f"Locked:  target {tracker_id[most_confident_target_id]}, type {class_id[most_confident_target_id]} with confidence {confidence[most_confident_target_id]}")
            
            
            
    # mode = ac.get_flight_mode()
    # if mode:
    #     print(mode)
    #     if mode == ac.tracking_mode:
            # print("Tracking.....")

    # detections = sv.Detections(
    #         xyxy=boxes,
    #         confidence=confidence,
    #         class_id=class_id,
    #         tracker_id=tracker_id)

    # print(tracker_id, confidence, boxes)
    return Gst.PadProbeReturn.OK


# -----------------------------------------------------------------------------------------------
# User Gstreamer Application
# -----------------------------------------------------------------------------------------------

# This class inherits from the hailo_rpi_common.GStreamerApp class
class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, args, user_data):
        # Call the parent class constructor
        super().__init__(args, user_data)
        # Additional initialization code can be added here
        # Set Hailo parameters these parameters should be set based on the model used
        self.batch_size = 2
        self.network_width = 640
        self.network_height = 640
        self.network_format = "RGB"
        nms_score_threshold = 0.3
        nms_iou_threshold = 0.45

        # Temporary code: new postprocess will be merged to TAPPAS.
        # Check if new postprocess so file exists
        new_postprocess_path = os.path.join(
            self.current_path, '../resources/libyolo_hailortpp_post.so')
        if os.path.exists(new_postprocess_path):
            self.default_postprocess_so = new_postprocess_path
        else:
            self.default_postprocess_so = os.path.join(
                self.postprocess_dir, 'libyolo_hailortpp_post.so')

        if args.hef_path is not None:
            self.hef_path = args.hef_path
        # Set the HEF file path based on the network
        elif args.network == "yolov6n":
            self.hef_path = os.path.join(
                self.current_path, '../resources/yolov6n.hef')
        elif args.network == "yolov8s":
            self.hef_path = os.path.join(
                self.current_path, '../resources/yolov8s_h8l.hef')
        elif args.network == "yolox_s_leaky":
            self.hef_path = os.path.join(
                self.current_path, '../resources/yolox_s_leaky_h8l_mz.hef')
        else:
            assert False, "Invalid network type"

        # User-defined label JSON file
        if args.labels_json is not None:
            self.labels_config = f' config-path={args.labels_json} '
        else:
            self.labels_config = ''

        self.app_callback = app_callback

        self.thresholds_str = (
            f"nms-score-threshold={nms_score_threshold} "
            f"nms-iou-threshold={nms_iou_threshold} "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        # Set the process title
        setproctitle.setproctitle("Hailo Detection App")

        # create a mavlink serial instance
        # comport = auto_connect(args.device)
        # self.master = mavutil.mavlink_connection(comport.device, baud=args.baudrate, source_system=args.source_system)

        # # wait for the heartbeat msg to find the system ID
        # wait_heartbeat(self.master)

        self.create_pipeline()

    def get_pipeline_string(self):
        if self.source_type == "rpi":
            source_element = (
                # "libcamerasrc name=src_0 auto-focus-mode=AfModeManual ! "
                # "libcamerasrc name=src_0 auto-focus-mode=2 ! "
                "libcamerasrc name=src_0 ! "
                f"video/x-raw, format={self.network_format}, width=1536, height=864 ! "
                + QUEUE("queue_src_scale")
                + "videoscale ! "
                f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, framerate=30/1 ! "
            )
        elif self.source_type == "usb":
            source_element = (
                f"v4l2src device={self.video_source} name=src_0 ! "
                "video/x-raw, width=640, height=480, framerate=30/1 ! "
            )
        else:
            source_element = (
                f"filesrc location=\"{self.video_source}\" name=src_0 ! "
                + QUEUE("queue_dec264")
                + " qtdemux ! h264parse ! avdec_h264 max-threads=2 ! "
                " video/x-raw, format=I420 ! "
            )
        source_element += QUEUE("queue_scale")
        source_element += "videoscale n-threads=2 ! "
        source_element += QUEUE("queue_src_convert")
        source_element += "videoconvert n-threads=3 name=src_convert qos=false ! "
        source_element += f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, pixel-aspect-ratio=1/1 ! "

        pipeline_string = (
            "hailomuxer name=hmux "
            + source_element
            + "tee name=t ! "
            + QUEUE("bypass_queue", max_size_buffers=20)
            + "hmux.sink_0 "
            + "t. ! "
            + QUEUE("queue_hailonet")
            + "videoconvert n-threads=3 ! "
            f"hailonet hef-path={self.hef_path} batch-size={self.batch_size} {self.thresholds_str} force-writable=true ! "
            + QUEUE("queue_hailofilter")
            + f"hailofilter so-path={self.default_postprocess_so} {self.labels_config} qos=false ! "
            + QUEUE("queue_hailotracker")
            + "hailotracker keep-tracked-frames=3 keep-new-frames=3 keep-lost-frames=3 ! "
            + QUEUE("queue_hmuc")
            + "hmux.sink_1 "
            + "hmux. ! "
            + QUEUE("queue_hailo_python")
            + QUEUE("queue_user_callback")
            + "identity name=identity_callback ! "
            + QUEUE("queue_hailooverlay")
            + "hailooverlay ! "
            + QUEUE("queue_videoconvert")
            + "videoconvert n-threads=3 qos=false ! "
            + QUEUE("queue_hailo_display")
            + f"fpsdisplaysink video-sink={self.video_sink} name=hailo_display sync={self.sync} text-overlay={self.options_menu.show_fps} signal-fps-measurements=true "
        )
        print(pipeline_string)
        return pipeline_string


if __name__ == "__main__":
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    parser = get_default_parser()
    # Add additional arguments here
    parser.add_argument(
        "--network",
        default="yolov6n",
        choices=['yolov6n', 'yolov8s', 'yolox_s_leaky'],
        help="Which Network to use, default is yolov6n",
    )
    parser.add_argument(
        "--hef-path",
        default=None,
        help="Path to HEF file",
    )
    parser.add_argument(
        "--labels-json",
        default=None,
        help="Path to costume labels JSON file",
    )
    parser.add_argument(
        "--ardu_device",
        required=True,
        help="serial device",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        help="master port baud rate", default=115200
    )
    parser.add_argument(
        "--source-system",
        dest='SOURCE_SYSTEM',
        type=int,
        default=255,
        help='MAVLink source system for this GCS'
    )
    parser.add_argument(
        "--tracking_mode",
        help="Autopilot mode when a target is tracked", default="GUIDED"
    )

    parser.add_argument(
        "--tracked_objects",
        type=str,
        help="Semicolon separated list of allowed objects, e.g. \"Armored;Cannon\"", default="Armored;Cannon"
    )
    parser.add_argument(
        "--min_confidence",
        type=float,
        help="Min confidence to lock", default=0.5
    )
    parser.add_argument(
        "--horizontal_fov",
        type=float,
        help="Camera horizontal FoV", default=60.0
    )
    parser.add_argument(
        "--vertical_fov",
        type=float,
        help="Camera vertical FoV", default=60.0
    )
    
    

    args = parser.parse_args()
    app = GStreamerDetectionApp(args, user_data)
    ac.configure(
        device=args.ardu_device,
        baudrate=args.baudrate,
        SOURCE_SYSTEM=args.SOURCE_SYSTEM,
        tracking_mode=args.tracking_mode,
        tracked_objects=args.tracked_objects,
        min_confidence=args.min_confidence
    )
    app.run()
