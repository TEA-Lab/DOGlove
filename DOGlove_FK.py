from pathlib import Path
import mujoco
import mujoco.viewer
# import numpy as np
from loop_rate_limiters import RateLimiter

# import mink

from glove_mcu import UDPReceiver

_HERE = Path(__file__).parent
_XML = _HERE / "DOGlove_meshes" / "DOGlove-v3.xml"




def get_body_pos(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    body_pos = data.xpos[body_id]
    return body_pos

def get_site_pos(model, data, site_name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    site_pos = data.site_xpos[site_id]
    return site_pos

def find_actuator_for_joint(model, joint_name):
    """Finds the actuator controlling the given joint."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    
    # Iterate through actuators to find the one controlling this joint
    for actuator_id in range(model.nu):  # `nu` is the number of actuators
        # `actuator_trnid` gives the joint or tendon that each actuator controls
        # actuator_trnid[actuator_id, 0] contains the joint/tendon ID
        if model.actuator_trnid[actuator_id][0] == joint_id:
            return actuator_id
    return None


def main():
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    receiver = UDPReceiver()
    receiver.start()

    try:
        with mujoco.viewer.launch_passive(
            model=model, data=data) as viewer:
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            mujoco.mj_forward(model, data)
            
            rate = RateLimiter(frequency=100.0)
            
            site_names = ['index_tip_site', 'middle_tip_site', 'ring_tip_site', 'little_tip_site', 'thumb_tip_site']
            
            joint_names = ['thumb_bend_1', 'thumb_bend_2', 'thumb_split', 'thumb_mcp', 
                           'index_bend_1', 'index_bend_2', 'index_split', 
                           'middle_bend_1', 'middle_bend_2', 'middle_split', 
                           'ring_bend_1', 'ring_bend_2', 'ring_split', 
                           'pinky_bend_1', 'pinky_bend_2', 'pinky_split',
                           'thumb_bend_3', 'index_bend_3', 'middle_bend_3', 'ring_bend_3', 'pinky_bend_3']
            # Initialize lists to store joint IDs and qpos indices
            joint_ids = []
            qpos_indices = []

            for joint_name in joint_names:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) # Get joint ID for each joint name
                qpos_index = model.jnt_qposadr[joint_id] # Get the corresponding qpos index
                # print(f"Joint Name: {joint_name}, Joint ID: {joint_id}, Qpos index: {qpos_index}")
                
                # Append joint_id and qpos_index to their respective lists
                joint_ids.append(joint_id)
                qpos_indices.append(qpos_index)

            # # Initialize the control value
            # ctrl_value = 0.1
            # increment = 0.1

            while viewer.is_running():
                mujoco.mj_step(model, data)
                
                for name in site_names:
                    site_pos = get_site_pos(model, data, name)
                    # print(f"{name}: {site_pos}")
                
                # print(f"Qpos: {data.qpos}")
                # data.qpos[qpos_index] = 0.5 # Set the joint angle
                # data.ctrl[joint_id] = 0.5 # Set the control value (this should drive the joint)
                # mujoco.mj_forward(model, data)
                # print(f"Index_dip joint angle: {data.qpos[qpos_index]}")

                # # Set the control value gradually, from 0.1 to 0.7
                # data.ctrl[joint_id] = ctrl_value
                # mujoco.mj_forward(model, data)

                # # Increment control value
                # ctrl_value += increment
                # if ctrl_value > 2.6:
                #     ctrl_value = 0.1  # Reset to start the cycle again

                # Get the most recent joint angles from the glove
                joints = receiver.get_most_recent_joints()
                if joints is not None:
                    # print(f"Index DIP: {round(joints[4], 2)}")
                    for i in range(len(joints)):
                        print(f"Joint[{i}]= {round(joints[i], 2)}")
                        data.ctrl[joint_ids[i]] = joints[i]
                    mujoco.mj_forward(model, data)
                
                # Visualize at fixed FPS.
                viewer.sync()
                rate.sleep()

    except KeyboardInterrupt:
        print("Program interrupted by user")

    finally:
        receiver.stop()
        print("UDP receiver stopped successfully")

if __name__ == '__main__':
    main()
