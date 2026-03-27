import time
import sys
import openvr
import math
import json
import numpy as np

class ViveTrackerModule():
    def __init__(self, configfile_path=None):
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()
        self.object_names = {"Tracking Reference":[],"HMD":[],"Controller":[],"Tracker":[]}
        self.devices = {}
        self.device_index_map = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                               openvr.k_unMaxTrackedDeviceCount)
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bDeviceIsConnected:
                self.add_tracked_device(i)

    def __del__(self):
        openvr.shutdown()

    def return_selected_devices(self, device_key=""):
        selected_devices = {}
        for key in self.devices:
            if device_key in key:
                selected_devices[key] = self.devices[key]
        return selected_devices

    def get_pose(self):
        return get_pose(self.vr)

    def poll_vr_events(self):
        event = openvr.VREvent_t()
        while self.vrsystem.pollNextEvent(event):
            if event.eventType == openvr.VREvent_TrackedDeviceActivated:
                self.add_tracked_device(event.trackedDeviceIndex)
            elif event.eventType == openvr.VREvent_TrackedDeviceDeactivated:
                if event.trackedDeviceIndex in self.device_index_map:
                    self.remove_tracked_device(event.trackedDeviceIndex)

    def add_tracked_device(self, tracked_device_index):
        i = tracked_device_index
        device_class = self.vr.getTrackedDeviceClass(i)
        if (device_class == openvr.TrackedDeviceClass_Controller):
            device_name = "controller_"+str(len(self.object_names["Controller"])+1)
            self.object_names["Controller"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Controller")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_HMD):
            device_name = "hmd_"+str(len(self.object_names["HMD"])+1)
            self.object_names["HMD"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"HMD")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_GenericTracker):
            device_name = "tracker_"+str(len(self.object_names["Tracker"])+1)
            self.object_names["Tracker"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Tracker")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_TrackingReference):
            device_name = "tracking_reference_"+str(len(self.object_names["Tracking Reference"])+1)
            self.object_names["Tracking Reference"].append(device_name)
            self.devices[device_name] = vr_tracking_reference(self.vr,i,"Tracking Reference")
            self.device_index_map[i] = device_name

    def remove_tracked_device(self, tracked_device_index):
        if tracked_device_index in self.device_index_map:
            device_name = self.device_index_map[tracked_device_index]
            self.object_names[self.devices[device_name].device_class].remove(device_name)
            del self.device_index_map[tracked_device_index]
            del self.devices[device_name]
        else:
            raise Exception("Tracked device index {} not valid. Not removing.".format(tracked_device_index))

    def rename_device(self,old_device_name,new_device_name):
        self.devices[new_device_name] = self.devices.pop(old_device_name)
        for i in range(len(self.object_names[self.devices[new_device_name].device_class])):
            if self.object_names[self.devices[new_device_name].device_class][i] == old_device_name:
                self.object_names[self.devices[new_device_name].device_class][i] = new_device_name

    def print_discovered_objects(self):
        for device_type in self.object_names:
            plural = device_type
            if len(self.object_names[device_type])!=1:
                plural+="s"
            print("Found "+str(len(self.object_names[device_type]))+" "+plural)
            for device in self.object_names[device_type]:
                if device_type == "Tracking Reference":
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", Mode "+self.devices[device].get_model()+
                          ", "+self.devices[device].get_model()+
                          ")")
                else:
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", "+")")

def update_text(txt):
    
    """Update the text in the same line on the console.

    Args:
        txt (str): The text to display.
    """
    sys.stdout.write('\r' + txt)
    sys.stdout.flush()

def convert_to_euler(pose_mat):
    """Convert a 3x4 position/rotation matrix to an x, y, z location and the corresponding Euler angles (in degrees).

    Args:
        pose_mat (list): A 3x4 position/rotation matrix.

    Returns:
        list: A list containing x, y, z, yaw, pitch, and roll values.
    """
    yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
    pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
    roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x, y, z, yaw, pitch, roll]

def convert_to_quaternion(pose_mat):

    """Convert a 3x4 position/rotation matrix to an x, y, z location and the corresponding quaternion.

    Args:
        pose_mat (list): A 3x4 position/rotation matrix.

    Returns:
        list: A list containing x, y, z, r_w, r_x, r_y, and r_z values.
    """
    # Calculate quaternion values
    r_w = math.sqrt(abs(1 + pose_mat[0][0] + pose_mat[1][1] + pose_mat[2][2])) / 2
    r_x = (pose_mat[2][1] - pose_mat[1][2]) / (4 * r_w)
    r_y = (pose_mat[0][2] - pose_mat[2][0]) / (4 * r_w)
    r_z = (pose_mat[1][0] - pose_mat[0][1]) / (4 * r_w)

    # Get position values
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]

    return [x, y, z, r_x, r_y, r_z, r_w]

#Define a class to make it easy to append pose matricies and convert to both Euler and Quaternion for plotting
class pose_sample_buffer():
    def __init__(self):
        self.i = 0
        self.index = []
        self.time = []
        self.x = []
        self.y = []
        self.z = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.r_w = []
        self.r_x = []
        self.r_y = []
        self.r_z = []

    def append(self,pose_mat,t):
        self.time.append(t)
        self.x.append(pose_mat[0][3])
        self.y.append(pose_mat[1][3])
        self.z.append(pose_mat[2][3])
        self.yaw.append(180 / math.pi * math.atan(pose_mat[1][0] /pose_mat[0][0]))
        self.pitch.append(180 / math.pi * math.atan(-1 * pose_mat[2][0] / math.sqrt(pow(pose_mat[2][1], 2) + math.pow(pose_mat[2][2], 2))))
        self.roll.append(180 / math.pi * math.atan(pose_mat[2][1] /pose_mat[2][2]))
        r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
        self.r_w.append(r_w)
        self.r_x.append((pose_mat[2][1]-pose_mat[1][2])/(4*r_w))
        self.r_y.append((pose_mat[0][2]-pose_mat[2][0])/(4*r_w))
        self.r_z.append((pose_mat[1][0]-pose_mat[0][1])/(4*r_w))

def get_pose(vr_obj):
    
    """Get the pose of a tracked device in the virtual reality system.

    Args:
        vr_obj (openvr object): An instance of the openvr object.

    Returns:
        list: A list of poses for each tracked device in the system.
    """
    return vr_obj.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)


class vr_tracked_device():
    
    def __init__(self, vr_obj, index, device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        self.T = np.eye(4, dtype=float)


    def get_serial(self):
        """Get the serial number of the tracked device."""
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    def get_model(self):
        """Get the model number of the tracked device."""
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)

    def get_battery_percent(self):
        """Get the battery percentage of the tracked device."""
        return self.vr.getFloatTrackedDeviceProperty(self.index, openvr.Prop_DeviceBatteryPercentage_Float)

    def is_charging(self):
        """Check if the tracked device is charging."""
        return self.vr.getBoolTrackedDeviceProperty(self.index, openvr.Prop_DeviceIsCharging_Bool)


    def sample(self, num_samples, sample_rate):
        """Sample the pose of the tracked device.

        Args:
            num_samples (int): Number of samples to collect.
            sample_rate (float): Rate at which to collect samples.

        Returns:
            PoseSampleBuffer: A buffer containing the collected pose samples.
        """
        interval = 1 / sample_rate
        rtn = pose_sample_buffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = get_pose(self.vr)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking, time.time() - sample_start)
            sleep_time = interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)
        return rtn

    def get_T(self, pose=None):
        pose_mat = self.get_pose_matrix()
        if pose_mat: # not None
            np_pose_mat = np.array(pose_mat)['m']
            self.T[:3,:] = np_pose_mat       
        return self.T

    def get_pose_euler(self, pose=None):
        """Get the pose of the tracked device in Euler angles.

        Args:
            pose (list, optional): The current pose of the device. If not provided, get_pose is called.

        Returns:
            tuple: Euler angles representing the pose, or None if the pose is not valid.
        """
        if pose is None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            print(convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking))
            return convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking)
            
        else:
            return None

    def get_pose_matrix(self, pose=None):
        """Get the pose matrix of the tracked device.

        Args:
            pose (list, optional): The current pose of the device. If not provided, get_pose is called.

        Returns:
            list: The pose matrix of the device, or None if the pose is not valid.
        """
        if pose is None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].mDeviceToAbsoluteTracking
        else:
            return None

    def get_velocity(self, pose=None):
        """Get the linear velocity of the tracked device.

        Args:
            pose (list, optional): The current pose of the device. If not provided, get_pose is called.

        Returns:
            tuple: The linear velocity of the device, or None if the pose is not valid.
        """
        if pose is None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vVelocity
        else:
            return None

    def get_angular_velocity(self, pose=None):
        # Get the angular velocity of the tracked device if its pose is valid.
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vAngularVelocity
        else:
            return None

    def get_pose_quaternion(self, pose=None):
        # Get the pose of the tracked device in the form of a quaternion if its pose is valid.
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return convert_to_quaternion(pose[self.index].mDeviceToAbsoluteTracking)
        else:
            return None

    def controller_state_to_dict(self, pControllerState):
        # Convert controller state data to a dictionary for easier use.
        d = {}
        # Fill dictionary with controller state data
        ...
        return d

    def get_controller_inputs(self):
        # Get the current state of the controller inputs.
        result, state = self.vr.getControllerState(self.index)
        return self.controller_state_to_dict(state)

    def trigger_haptic_pulse(self, duration_micros=1000, axis_id=0):
        # Trigger a haptic pulse on the controller.
        self.vr.triggerHapticPulse(self.index ,axis_id, duration_micros)

class vr_tracking_reference(vr_tracked_device):
    def get_mode(self):
        # Get the mode of the tracking reference.
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_ModeLabel_String).decode('utf-8').upper()

    def sample(self,num_samples,sample_rate):
        # Warn the user that sampling a tracking reference is not useful, as they do not move.
        print("Tracker static!")
