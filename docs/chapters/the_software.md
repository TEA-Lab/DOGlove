# Software

This section outlines the key components of the software stack for DOGlove, including communication protocols, servo control, feedback mechanisms, and calibration.

## Communication Between PC and Main Board

- **UART Package** – Used for low-level serial communication.  
- **Vibration Package** – Responsible for encoding and transmitting vibration feedback.  
- **UDP** – Used for real-time data streaming between the glove and host computer.

## Servo Data Path

- **UDP** – Servo position commands and sensor data are transmitted over UDP for low-latency control.

## Haptic + Force Feedback

- **Force Sensor Readings** – Captures contact forces at the robot fingertips.
- **Force Feedback Activation Threshold** –  
  When the measured force exceeds a preset threshold, force feedback is enabled by turning on `servo.torque`.

  - To adjust the *stiffness* of the force feedback, modify the `Kp` gain parameter.

- **Vibration Activation Threshold** –  
  When a separate threshold is crossed, the MCU sends vibration commands to the glove.

  - You can adjust the **vibration pattern** and **duration** using the TI vibration library.

## Forward and Inverse Kinematics (FK & IK)

- Add the **site name** in the corresponding `.xml` file to support FK/IK calculations for the pinky finger.

## Calibration: Scale and Shift

To ensure realistic hand motion mapping, we apply scale and shift adjustments:

- **Finger Length Discrepancy → Scale Factor**  
  Scale the glove’s finger motion to match the relative lengths of the robotic hand.

- **Finger Distance Discrepancy → Shift Offset**  
  Adjust for base position differences between fingers to ensure accurate positioning.

The goal is simple: when the real thumb and index finger touch, the robot’s thumb and index should touch as well.
