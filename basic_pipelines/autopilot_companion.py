from pymavlink import mavutil
import time
import math
from pymavlink import quaternion
import os
import ctypes as ct


def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" %
          (m.target_system, m.target_component))


class AngularControl(ct.Structure):
    _fields_ = [
        ("control_output_x", ct.c_float),
        ("control_output_y", ct.c_float),
        ("control_output_z", ct.c_float),
    ]


class AutipilotCompanion:

    state_wait = "wait"
    state_hold = "hold"

    typemask_ypr_throttle = 0b00111000
    type_mask_ypr = 0b01111000

    def __init__(self):

        self.target_id_locked = -1
        self.boot_time = time.time()

    def close(self):

        print("Closing the vehicle...")
        cac_handle = self.cac._handle
        del self.cac
        self._dlclose_func(cac_handle)
        self.master.close()

    def configure(self, 
                device, 
                baudrate=115200, 
                SOURCE_SYSTEM=255, 
                tracking_mode="GUIDED", 
                tracked_objects="Armored;Cannon", 
                min_confidence=0.5, 
                angularcs_lib_path="resources/angularcs.so",
                horizontal_fov=60,
                vertical_fov=60,
                timesample=0.1
        ):

        self.tracking_mode = tracking_mode
        self.min_confidence = min_confidence
        self.tracked_objects = tracked_objects
        self.horizontal_fov = horizontal_fov
        self.vertical_fov = vertical_fov
        self.timesample = timesample
        
        self.angularcs_lib_path = os.path.join(os.getcwd(), angularcs_lib_path)

        if not os.path.exists(self.angularcs_lib_path):
            raise FileNotFoundError(
                f"Angularcs lib not found: {self.angularcs_lib_path}")

        print(f"Loading shared lib: {self.angularcs_lib_path}")

        self.cac = ct.CDLL(self.angularcs_lib_path)
        self.cac.control_angular_motion_ex.argtypes = [
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_uint32,
            ct.c_uint32,
            ct.c_uint32,
            ct.c_uint32,
        ]
        self.cac.control_angular_motion_ex.restype = AngularControl

        self._dlclose_func = ct.cdll.LoadLibrary('').dlclose
        self._dlclose_func.argtypes = [ct.c_void_p]

        self.master = mavutil.mavlink_connection(
            device, baud=baudrate, source_system=SOURCE_SYSTEM)

        self.__wait_conn()

    def get_flight_mode(self):

        mode = ""

        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)

        return mode

    def __wait_conn(self):
        """
        Sends a ping to stabilish the UDP communication and awaits for a response
        """
        msg = None
        while not msg:
            self.master.mav.ping_send(
                int(time.time() * 1e6),  # Unix time in microseconds
                0,  # Ping number
                0,  # Request ping of all systems
                0  # Request ping of all components
            )
            msg = self.master.recv_match()
            print(f"Connecting message: {msg}")
            time.sleep(0.5)

    def lock_target(self, target_id):

        self.target_id_locked = target_id

    def is_tracked(self, tracker_id: int) -> bool:
        """Checks if input id is already tracked 

        Args:
            tracker_id (int): id from tracker

        Returns:
            bool: True if it is already tracked
        """

        return self.__locked_id == tracker_id

    def has_tracked_id(self, detections) -> int:

        pass

    def process_video_frame(self, class_ids: dict, confidences: dict, boxes: dict, video_frame_size: tuple):
        """Processes frames 

        Args:
            class_ids (dict): list of class ids
            confidences (dict): list of confidences
            boxes (dict): list of bboxes in the format [xmin,ymin.xmax,ymax]
            frame_size (tuple): a tuple of w and h (w,h)
        """
        # checking if the locked object id in the current detections
        if self.target_id_locked in class_ids.keys():
            
            # calculating x, y bias 
            target_xy_bias = self.xy_bias(
                tuple((float(i) for i in boxes[self.target_id_locked])))
            print(f"Bias xy: {target_xy_bias}")

            # calculating angles in radians
            velocities = self.visual_pid_wrapper(
                target_xy_bias[0], target_xy_bias[1])    
            
            print(f"Velocities yaw, pitch, roll: {velocities}")

            # set flight angles
            self.__set_attitude_target(
                roll=velocities['roll'],
                pitch=velocities['pitch'],
                yaw=velocities['yaw'],
                throttle=velocities['throttle'])

        else:
            # if detection.get_confidence() < ac.min_confidence:
            #     continue
            self.target_id_locked = 10

    @staticmethod
    def xy_bias(bbox_xyxy: tuple[float, float, float, float], w=1.0, h=1.0) -> list:
        return [(bbox_xyxy[0] + bbox_xyxy[2]) / 2 - (w / 2), (bbox_xyxy[1] + bbox_xyxy[3]) / 2 - (h / 2)]

    def visual_pid_wrapper(self, x_bias, y_bias) -> dict:

        velocities = {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': 0
        }
        
        pid_velocities = self.__calculate_velocities(
            x_bias, y_bias)

        # Dummy calculation for testing
        # velocities['roll'] = -90 * x_bias
        # velocities['yaw'] = -45 * x_bias
        
        velocities['yaw'] = pid_velocities[0]
        velocities['pitch'] = pid_velocities[1]
        velocities['roll'] = pid_velocities[2]
        

        return velocities

    def __set_attitude_target(self, roll: float, pitch: float, yaw: float, throttle=0, tolerance=10):
        """Set flight parameters for a plane: yaw, pitch, roll, throttle

        Args:
            roll (float): roll angle in rad
            pitch (float): pitch angle in rad
            yaw (float): yaw angle in rad
            throttle (int, optional): a throttle level. Defaults to 0.
            tolerance (int, optional): a tolerance level (is not used in the current implementation). Defaults to 10.
        """

        try:
            m = self.master.recv_match(
                type='ATTITUDE',
                blocking=True,
                timeout=0.1)
            
            if m is None:
                return

            # attitude in radians:
            q = quaternion.Quaternion([roll, pitch, yaw])
            self.master.mav.set_attitude_target_send(
                int(1e3 * (time.time() - self.boot_time)),
                self.master.target_system,
                self.master.target_component,
                self.typemask_ypr_throttle,
                q,
                0,  # roll rate, not used in AP
                0,  # pitch rate, not used in AP
                0,  # yaw rate, not used in AP
                throttle)
        except Exception as e:
            print(
                f"Error setting attitude: \n   - roll: {roll}\n   - pitch: {pitch}\n   - yaw: {yaw}\n   - throttle: {throttle} \n {e}")

    def __calculate_velocities(self, erx: float, ery: float, erz=0.0, w=1, h=1) -> tuple:
        """Calculates required velocities among x (yaw), y (pitch), z (roll) axis

        Args:
            erx (int): target direction error among x
            ery (int): target direction error among y
            ery (int): target direction error among z

        Returns:
            tuple: tuple of required velocities among x, y, z axis (yaw, pitch, roll)
        """

        try:

            velocity = self.cac.control_angular_motion_ex(
                float(erx),
                float(ery),
                float(erz),
                self.timesample,
                w,
                h,
                self.horizontal_fov,
                self.vertical_fov,
            )
            return (
                velocity.control_output_x, # yaw
                velocity.control_output_y, # pitch
                velocity.control_output_z, # roll
            )
        except Exception as er:
            print(f"Error calling shared lib: {er}")
            return 0, 0, 0
