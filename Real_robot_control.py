#!/usr/bin/env python3
import time
import roslibpy
import numpy as np

current_pos = None  # Global variable to store current joint positions
# ------------------- Gripper Functions -------------------
def call_service(client, service_name):
    """Call a std_srvs/Trigger service and print the response."""
    service = roslibpy.Service(client, service_name, 'std_srvs/Trigger')
    request = roslibpy.ServiceRequest({})  # Trigger service takes no arguments

    print(f"[ROS] Calling service: {service_name}")
    result = service.call(request)
    print(f"[ROS] Response: success={result['success']}, message='{result['message']}'")
    return result

def open_gripper(client):
    """Open the gripper by calling the /onrobot/open service."""
    return call_service(client, '/onrobot/open')


def close_gripper(client):
    """Close the gripper by calling the /onrobot/close service."""
    return call_service(client, '/onrobot/close')


# ------------------- UR3 Motion Functions -------------------
def joint_state_cb(message):
    global current_pos
    current_pos = list(message['position'])


def move_ur_joint_positions(client, joint_positions, duration=5.0):
    """Move UR3 to target joint positions using JointTrajectory."""
    global current_pos

    # Subscribe to joint states to get the current position
    listener = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
    listener.subscribe(joint_state_cb)

    # Wait until we receive a joint state
    print("[ROS] Waiting for current joint state...")
    start_time = time.time()
    while current_pos is None and time.time() - start_time < 5.0:
        time.sleep(0.05)
    if current_pos is None:
        raise RuntimeError("No joint state received from /ur/joint_states")

    print(f"[ROS] Current joint positions: {current_pos}")

    # Joint names for UR3
    joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    trajectory_msg = {
        'joint_names': joint_names,
        'points': [
            {
                'positions': current_pos,
                'time_from_start': {'secs': 0, 'nsecs': 0}
            },
            {
                'positions': joint_positions,
                'time_from_start': {
                    'secs': int(duration),
                    'nsecs': int((duration - int(duration)) * 1e9)
                }
            }
        ]
    }

    # Publish trajectory
    topic = roslibpy.Topic(
        client,
        '/ur/scaled_pos_joint_traj_controller/command',
        'trajectory_msgs/JointTrajectory'
    )
    topic.advertise()
    topic.publish(roslibpy.Message(trajectory_msg))
    print("[ROS] Trajectory published.")

    # Wait for motion to complete
    time.sleep(duration + 1.0)

    topic.unadvertise()
    listener.unsubscribe()


# ------------------- Main Workflow -------------------
def main():
    ros_host = '192.168.27.1'  # Replace with your ROS bridge IP
    ros_port = 9090

    client = roslibpy.Ros(host=ros_host, port=ros_port)
    try:
        client.run()
        if not client.is_connected:
            raise RuntimeError("Failed to connect to rosbridge.")

        print("[ROS] Connected to rosbridge.")

        # Step 1: Open gripper
        call_service(client, '/onrobot/open')
        time.sleep(2)

        home_joint_positions = [np.deg2rad(84), np.deg2rad(-103.90), np.deg2rad(21.12), np.deg2rad(-98.65), np.deg2rad(-80.10), np.deg2rad(186.10)]
        move_ur_joint_positions(client, home_joint_positions, duration=5.0)

        first_position = [np.deg2rad(76.51), np.deg2rad(-64.96), np.deg2rad(69.73), np.deg2rad(-97.88), np.deg2rad(-96.18), np.deg2rad(165.59)]
        
        move_ur_joint_positions(client, first_position, duration=5.0)

        call_service(client, '/onrobot/close')
        time.sleep(2)

        second_position = [np.deg2rad(93.59), np.deg2rad(-66.14), np.deg2rad(68.72), np.deg2rad(-94.25), np.deg2rad(-92.03), np.deg2rad(182.63)]
        move_ur_joint_positions(client, second_position, duration=5.0)
        
        move_ur_joint_positions(client, home_joint_positions, duration=5.0)
        
        call_service(client, '/onrobot/open')
        time.sleep(2)

        third_position = [np.deg2rad(72.66), np.deg2rad(-80.20), np.deg2rad(90.61),np.deg2rad(-101.64),np.deg2rad(-94.88), np.deg2rad(164.78)]
        move_ur_joint_positions(client, third_position, duration=5.0)
        
        fourth_position = [np.deg2rad(72.66), np.deg2rad(-80.20), np.deg2rad(90.61),np.deg2rad(-101.64),np.deg2rad(-94.88), np.deg2rad(164.78)]
        move_ur_joint_positions(client, fourth_position, duration=5.0)
        
        call_service(client, '/onrobot/open')
        time.sleep(2)


    finally:
        client.terminate()
        print("[ROS] Disconnected.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[APP] Interrupted by user.")
