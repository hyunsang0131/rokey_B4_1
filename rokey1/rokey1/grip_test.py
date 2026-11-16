# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
import time
from onrobot_rg_msgs.srv import SetCommand

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            wait,
        )

        from DR_common2 import posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    # RG2 그리퍼 서비스 (virtual: gripper_virtual_node / real: OnRobot 드라이버)
    # 디지털 출력만으로는 RViz의 RG2가 움직이지 않으므로 서비스도 함께 호출
    gripper_client = node.create_client(SetCommand, "/onrobot/sendCommand")

    def send_gripper(cmd):
        if not gripper_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn("/onrobot/sendCommand 서비스 없음 - 그리퍼 서비스 호출 생략")
            return
        req = SetCommand.Request()
        req.command = cmd
        future = gripper_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        res = future.result()
        if res is None or not res.success:
            node.get_logger().warn(f"gripper '{cmd}' failed: {res.message if res else 'timeout'}")

    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        send_gripper("c")
        # wait(1)
        time.sleep(1)

    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        send_gripper("o")
        wait(1)

    homej = posj([0, 0, 90, 0, 90, 0])

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")

            grip()
            release()
            wait(1)

        node.get_logger().info("Gripper Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
