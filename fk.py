from pathlib import Path

import mujoco
import mujoco.viewer
from loop_rate_limiters import RateLimiter
from udp_receiver import UDPReceiver


_XML = Path(__file__).parent / "DOGlove_meshes" / "DOGlove-v3.xml"

FK_JOINT_NAMES = (
    "thumb_bend_1",
    "thumb_bend_2",
    "thumb_split",
    "thumb_mcp",
    "index_bend_1",
    "index_bend_2",
    "index_split",
    "middle_bend_1",
    "middle_bend_2",
    "middle_split",
    "ring_bend_1",
    "ring_bend_2",
    "ring_split",
    "pinky_bend_1",
    "pinky_bend_2",
    "pinky_split",
    "thumb_bend_3",
    "index_bend_3",
    "middle_bend_3",
    "ring_bend_3",
    "pinky_bend_3",
)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        for joint_name in FK_JOINT_NAMES
    ]

    receiver = UDPReceiver()
    receiver.start()

    try:
        with mujoco.viewer.launch_passive(model=model, data=data) as viewer:
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            mujoco.mj_forward(model, data)

            rate = RateLimiter(frequency=100.0)

            while viewer.is_running():
                mujoco.mj_step(model, data)

                joints = receiver.get_most_recent_joints()
                if joints is not None:
                    for i, joint_value in enumerate(joints):
                        print(f"Joint[{i}]= {round(joint_value, 2)}")
                        if i < len(joint_ids):
                            data.ctrl[joint_ids[i]] = joint_value
                    mujoco.mj_forward(model, data)

                viewer.sync()
                rate.sleep()

    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        receiver.stop()
        print("UDP receiver stopped successfully")


if __name__ == "__main__":
    main()
