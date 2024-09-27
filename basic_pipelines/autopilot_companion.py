from pymavlink import mavutil
import time
import math

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))


class AutipilotCompanion:   
    
    
    def __init__(self):
        
        self.target_id_locked = -1
    
    def configure(self, device, baudrate=115200, SOURCE_SYSTEM=255, tracking_mode="GUIDED", allowed_objects="Armored;Cannon", min_confidence=0.5):

        self.tracking_mode = tracking_mode
        # self.allowed_objects = allowed_objects
        
        self.min_confidence = min_confidence
        
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
    
    
    def process_frame(self, class_ids: dict, confidences: dict, boxes: dict, frame_size: tuple):
        """Processes frames 

        Args:
            class_ids (dict): list of class ids
            confidences (dict): list of confidences
            boxes (dict): list of bboxes in the format [xmin,ymin.xmax,ymax]
            frame_size (tuple): a tuple of w and h (w,h)
        """
        
        if self.target_id_locked in class_ids.keys():
            
            target_xy_bias = self.xy_bias(tuple((float(i) for i in boxes[self.target_id_locked])))
            print(target_xy_bias)
            
            
        else:
            # if detection.get_confidence() < ac.min_confidence:
            #     continue
            self.target_id_locked = 5

    @staticmethod
    def xy_bias(bbox_xyxy: tuple[float, float, float, float], w= 1.0, h = 1.0) -> list:
        return [(bbox_xyxy[0] + bbox_xyxy[2]) / 2 - (w / 2), (bbox_xyxy[1] + bbox_xyxy[3]) / 2 - (h / 2)]


    def flight_controller_wrapper(self, x_bias, y_bias) -> list:

        velocities = []

        return velocities
