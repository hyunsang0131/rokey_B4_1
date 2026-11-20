#!/usr/bin/env python3
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MoveStop,
    SetCtrlBoxDigitalOutput,
    SetToolDigitalOutput,
    GetToolForce,
    GetCurrentPosx,
)

ON, OFF = 1, 0


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def normalize_deg(angle_deg: float) -> float:
    """각도를 -180 ~ +180도 범위로 정규화"""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


class DsrRg2MultiProbingNode(Node):
    def __init__(self):
        super().__init__('dsr_tactile_probing_node')

        self.marker_pub = self.create_publisher(MarkerArray, '/tactile_sim_markers', 10)
        self.gripper_joint_pub = self.create_publisher(JointState, '/gripper_joint_states', 10)
        self.gripper_joint_pub_ns = self.create_publisher(JointState, '/dsr01/gripper_joint_states', 10)
        self.gripper_cmd_pub = self.create_publisher(Float64, '/gripper_command', 10)

        self.base_frame = 'base_link'
        self.ee_frame = 'rg2_tcp'
        self.gripper_length_mm = 150.0
        # 목표 물체 규격: 가로(짧은 변) 41mm, 세로(긴 변) 64mm, 높이 20mm
        self.matrix_size = [0.064, 0.041, 0.020]

        # 사전에 입력받은 대략적인 물체 중심 좌표 (m -> mm 변환)
        zone_centers_m = [(0.34481, 0.0)]
        self.objects = [{
            "id": 0,
            "zone_center_mm": [zone_centers_m[0][0] * 1000.0, zone_centers_m[0][1] * 1000.0],
            "est_center": np.array([zone_centers_m[0][0], zone_centers_m[0][1], self.matrix_size[2] / 2.0]),
            "est_yaw": 0.0,
            "state": "waiting",
            "remembered_contacts": [],
            "remembered_pose": None,
        }]

        self.remembered_object_pose = None
        self.active_obj_idx = 0

        # URDF 모델 기준 완전 닫힘 각도(0.68) / 활짝 열림 각도(0.0)
        self.close_joint_angle = 0.68
        self.open_joint_angle = 0.0
        self.gripper_angle = self.close_joint_angle
        self.is_probing_mode = True

        self.contact_points = []
        self.coarse_paths = []
        self.fine_paths = []

        # 접촉 판정 외력 임계값: 3.8 N
        self.force_threshold_xy = 4.5
        self.force_threshold_z = 4.5

        self.baseline_wrench = np.zeros(6)

        self.create_timer(0.1, self.publish_visuals)

        # 서비스 자동 탐색
        self.get_logger().info("활성화된 두산 제어기 서비스 탐색 중...")
        motion_prefix = None
        aux_prefix = None
        io_prefix = None

        while rclpy.ok() and motion_prefix is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            for srv_name, _ in self.get_service_names_and_types():
                if 'move_joint' in srv_name and not srv_name.endswith('move_jointx'):
                    motion_prefix = srv_name.rsplit('/', 1)[0]
                if srv_name.endswith('/aux_control/get_tool_force'):
                    aux_prefix = srv_name.rsplit('/', 1)[0]
                if srv_name.endswith('/io/set_ctrl_box_digital_output'):
                    io_prefix = srv_name.rsplit('/', 1)[0]

        if aux_prefix is None:
            aux_prefix = motion_prefix.replace('/motion', '/aux_control')

        self.cli_movej = self.create_client(MoveJoint, f"{motion_prefix}/move_joint")
        self.cli_movel = self.create_client(MoveLine, f"{motion_prefix}/move_line")
        self.cli_stop = self.create_client(MoveStop, f"{motion_prefix}/move_stop")
        self.cli_force = self.create_client(GetToolForce, f"{aux_prefix}/get_tool_force")
        self.cli_posx = self.create_client(GetCurrentPosx, f"{aux_prefix}/get_current_posx")

        for cli in [self.cli_movej, self.cli_movel, self.cli_stop, self.cli_force, self.cli_posx]:
            cli.wait_for_service()

        self.cli_do = None
        self.cli_tool_do = None
        if io_prefix:
            self.cli_do = self.create_client(SetCtrlBoxDigitalOutput, f"{io_prefix}/set_ctrl_box_digital_output")
            self.cli_do.wait_for_service()
            self.cli_tool_do = self.create_client(SetToolDigitalOutput, f"{io_prefix}/set_tool_digital_output")
            if not self.cli_tool_do.wait_for_service(timeout_sec=1.0):
                self.cli_tool_do = None

        self.get_logger().info(
            f"두산 제어기 서비스 연결 완료! (IO 연결: CtrlBox={self.cli_do is not None}, ToolIO={self.cli_tool_do is not None})"
        )

    def get_current_wrench(self):
        req = GetToolForce.Request()
        req.ref = 0
        future = self.cli_force.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
        res = future.result()
        if res is not None and res.success and len(res.tool_force) >= 6:
            return np.array(res.tool_force[:6], dtype=float)
        return np.zeros(6)

    def get_current_tcp_pos(self):
        req = GetCurrentPosx.Request()
        req.ref = 0
        future = self.cli_posx.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
        res = future.result()
        if res is not None and res.success and len(res.task_pos_info) > 0:
            return np.array(res.task_pos_info[0].data[:6], dtype=float)
        return np.zeros(6)

    def calibrate_force_baseline(self, num_samples=5):
        samples = [self.get_current_wrench() for _ in range(num_samples)]
        self.baseline_wrench = np.mean(samples, axis=0)

    def stop_robot_immediately(self):
        req = MoveStop.Request()
        req.stop_mode = 1
        future = self.cli_stop.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)

    def _set_do(self, index: int, value: int):
        """컨트롤박스 DO와 툴 IO 모두 출력하여 RG2 그리퍼가 확실히 반응하도록 전송"""
        if self.cli_do is not None:
            req = SetCtrlBoxDigitalOutput.Request()
            req.index = int(index)
            req.value = int(value)
            future = self.cli_do.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if self.cli_tool_do is not None:
            t_req = SetToolDigitalOutput.Request()
            t_req.index = int(index)
            t_req.value = int(value)
            t_future = self.cli_tool_do.call_async(t_req)
            rclpy.spin_until_future_complete(self, t_future, timeout_sec=1.0)

    def close_gripper_pulse(self, wait_sec=1.5):
        """
        RG2 그리퍼를 완전히 닫기 위해 OFF -> ON 상승 엣지(Rising Edge) 펄스를 인가하고
        완전히 닫힐 때까지 대기
        """
        self._set_do(1, OFF)
        self._set_do(2, ON)
        time.sleep(0.15)
        self._set_do(1, ON)
        self._set_do(2, OFF)

        self.gripper_angle = self.close_joint_angle
        cmd_msg = Float64()
        cmd_msg.data = 0.0  # 완전 닫힘 폭 0mm
        self.gripper_cmd_pub.publish(cmd_msg)
        self.publish_visuals()
        time.sleep(wait_sec)

    def grip_for_probing(self, wait_sec=1.5):
        """물체 탐지용 그리퍼 완전 닫기 (탐지 전 구간 동안 닫힌 상태 유지)"""
        self.is_probing_mode = True
        self.get_logger().info("[그리퍼 닫기] 물체 탐지 시작 전 그리퍼를 완전히 닫습니다...")
        self.close_gripper_pulse(wait_sec=wait_sec)
        self.get_logger().info("[그리퍼 닫힘 완료] 닫힌 상태로 물체 위치 추정을 진행합니다.")

    def release_wide(self, wait_sec=1.5):
        """최종 파지 직전 상단에서만 그리퍼를 완전히 벌림"""
        self.is_probing_mode = False
        self.get_logger().info(f"[그리퍼 열기] Z축 상단에서 그리퍼를 완전히 벌립니다 (대기: {wait_sec}초)")
        self._set_do(1, OFF)
        self._set_do(2, OFF)
        time.sleep(0.15)
        self._set_do(1, OFF)
        self._set_do(2, ON)

        self.gripper_angle = self.open_joint_angle
        cmd_msg = Float64()
        cmd_msg.data = 110.0  # 최대 열림 폭 110mm
        self.gripper_cmd_pub.publish(cmd_msg)
        self.publish_visuals()
        time.sleep(wait_sec)

    def close_gripper_full_for_grasp(self, wait_sec=1.8):
        """물체 위치에 도달한 후 그리퍼를 완전히 닫아 물체를 파지"""
        self.is_probing_mode = False
        self.get_logger().info(f"[그리퍼 완전 닫기] 물체 위치 도달 -> 그리퍼를 완전히 닫습니다 (대기: {wait_sec}초)")
        self.close_gripper_pulse(wait_sec=wait_sec)
        self.get_logger().info("[그리퍼 완전 닫힘 완료] 물체 파지가 완료되었습니다.")

    def send_movej(self, pos_deg, vel=60.0, acc=60.0):
        req = MoveJoint.Request()
        req.pos = [float(x) for x in pos_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_movel(self, pos_mm_deg, vel=60.0, acc=60.0, radius=0.0, sync_type=0):
        req = MoveLine.Request()
        req.pos = [float(x) for x in pos_mm_deg]
        req.vel = [float(vel), float(vel)]
        req.acc = [float(acc), float(acc)]
        req.time = 0.0
        req.radius = float(radius)
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = int(sync_type)
        future = self.cli_movel.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def reset_path_markers(self):
        self.contact_points = []
        self.coarse_paths = []
        self.fine_paths = []
        delete_array = MarkerArray()
        for ns, start_id, count in [("contacts", 1, 1), ("coarse_paths", 10, 10), ("fine_paths", 50, 10)]:
            for i in range(count):
                m = Marker()
                m.header.frame_id = self.base_frame
                m.ns = ns
                m.id = start_id + i
                m.action = Marker.DELETE
                delete_array.markers.append(m)
        self.marker_pub.publish(delete_array)

    def publish_visuals(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        js = JointState()
        js.header.stamp = now
        js.name = [
            'rg2_finger_joint', 'rg2_left_inner_knuckle_joint', 'rg2_left_inner_finger_joint',
            'rg2_right_outer_knuckle_joint', 'rg2_right_inner_knuckle_joint', 'rg2_right_inner_finger_joint'
        ]
        a = self.close_joint_angle if self.is_probing_mode else self.gripper_angle
        js.position = [a, -a, a, -a, -a, a]
        self.gripper_joint_pub.publish(js)
        self.gripper_joint_pub_ns.publish(js)

        for idx, obj in enumerate(self.objects):
            m_box = Marker()
            m_box.header.stamp = now
            m_box.ns = "target_objects"
            m_box.id = idx
            m_box.type = Marker.CUBE
            m_box.action = Marker.ADD
            m_box.scale.x, m_box.scale.y, m_box.scale.z = self.matrix_size
            if obj["state"] == "grasped":
                m_box.header.frame_id = self.ee_frame
                m_box.pose.position = Point(x=0.0, y=0.0, z=0.0)
                m_box.pose.orientation = euler_to_quaternion(0.0, 0.0, 0.0)
                m_box.color = ColorRGBA(r=0.1, g=0.9, b=1.0, a=0.95)
            else:
                m_box.header.frame_id = self.base_frame
                m_box.pose.position = Point(
                    x=float(obj["est_center"][0]),
                    y=float(obj["est_center"][1]),
                    z=float(obj["est_center"][2])
                )
                m_box.pose.orientation = euler_to_quaternion(0.0, 0.0, obj["est_yaw"])
                m_box.color = ColorRGBA(r=0.90, g=0.35, b=0.35, a=1.0)
            marker_array.markers.append(m_box)

        m_pts = Marker()
        m_pts.header.frame_id = self.base_frame
        m_pts.header.stamp = now
        m_pts.ns = "contacts"
        m_pts.id = 1
        m_pts.type = Marker.SPHERE_LIST
        m_pts.action = Marker.ADD
        m_pts.scale.x = m_pts.scale.y = m_pts.scale.z = 0.008
        m_pts.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
        for pt in self.contact_points:
            m_pts.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
        marker_array.markers.append(m_pts)

        for idx, path in enumerate(self.coarse_paths):
            m_line = Marker()
            m_line.header.frame_id = self.base_frame
            m_line.header.stamp = now
            m_line.ns = "coarse_paths"
            m_line.id = 10 + idx
            m_line.type = Marker.LINE_STRIP
            m_line.action = Marker.ADD
            m_line.scale.x = 0.0025
            m_line.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.75)
            for p in path:
                m_line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            marker_array.markers.append(m_line)

        for idx, path in enumerate(self.fine_paths):
            m_line = Marker()
            m_line.header.frame_id = self.base_frame
            m_line.header.stamp = now
            m_line.ns = "fine_paths"
            m_line.id = 50 + idx
            m_line.type = Marker.LINE_STRIP
            m_line.action = Marker.ADD
            m_line.scale.x = 0.0035
            m_line.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.95)
            for p in path:
                m_line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            marker_array.markers.append(m_line)

        self.marker_pub.publish(marker_array)


def save_top_view_plot(contacts, center_xy_mm, length_mm, width_mm, long_axis_yaw_deg, output_path):
    try:
        import matplotlib
    except ImportError as exc:
        raise RuntimeError(
            "Top-view plotting requires Matplotlib; on Ubuntu install it with "
            "`sudo apt install python3-matplotlib`."
        ) from exc

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    center = np.asarray(center_xy_mm, dtype=float)
    angle_rad = math.radians(long_axis_yaw_deg)
    long_axis = np.array([math.cos(angle_rad), math.sin(angle_rad)])
    short_axis = np.array([-math.sin(angle_rad), math.cos(angle_rad)])
    half_length = length_mm / 2.0
    half_width = width_mm / 2.0
    corners = np.array([
        center - long_axis * half_length - short_axis * half_width,
        center + long_axis * half_length - short_axis * half_width,
        center + long_axis * half_length + short_axis * half_width,
        center - long_axis * half_length + short_axis * half_width,
    ])

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.add_patch(Polygon(corners, closed=True, facecolor="cornflowerblue", alpha=0.25,
                         edgecolor="navy", linewidth=2, label="Estimated cube"))
    ax.scatter(center[0], center[1], marker="x", s=90, color="navy", label="Estimated center")

    plotted_contacts = []
    for index, contact in enumerate(contacts, start=1):
        point = np.asarray(contact["xy"], dtype=float)
        if point.size < 2:
            continue
        point = point[:2]
        plotted_contacts.append(point)
        ax.scatter(point[0], point[1], s=45, zorder=3)
        ax.annotate(
            f"{index}: {contact['label']}\n({point[0]:.1f}, {point[1]:.1f})",
            (point[0], point[1]),
            xytext=(5, 6),
            textcoords="offset points",
            fontsize=8,
        )

    bounds = np.vstack([corners, np.asarray(plotted_contacts)]) if plotted_contacts else corners
    padding = max(length_mm, width_mm) * 0.55
    ax.set_xlim(float(bounds[:, 0].min() - padding), float(bounds[:, 0].max() + padding))
    ax.set_ylim(float(bounds[:, 1].min() - padding), float(bounds[:, 1].max() + padding))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Base X (mm)")
    ax.set_ylabel("Base Y (mm)")
    ax.set_title(
        f"Estimated top view: {length_mm:.0f} x {width_mm:.0f} mm, "
        f"long-axis yaw {long_axis_yaw_deg:.1f} deg"
    )
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    try:
        fig.savefig(output_path, dpi=180, bbox_inches="tight")
    finally:
        plt.close(fig)

    return output_path


def save_force_time_plot(sample_times, force_samples, contact_force_samples,
                         threshold, label, sequence, contact_detected, output_dir):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    safe_label = "".join(
        character if character.isascii() and character.isalnum() else "_"
        for character in label
    ).strip("_") or "probe"
    output_path = output_dir / f"force_{sequence:03d}_{time.time_ns()}_{safe_label}.png"

    forces = np.asarray(force_samples, dtype=float).reshape((-1, 3))
    fig, ax = plt.subplots(figsize=(10, 6))
    if sample_times:
        ax.plot(sample_times, forces[:, 0], label="Fx (baseline corrected)")
        ax.plot(sample_times, forces[:, 1], label="Fy (baseline corrected)")
        ax.plot(sample_times, forces[:, 2], label="Fz (baseline corrected)")
        ax.plot(sample_times, contact_force_samples, linewidth=2, label="Contact-direction force")
        ax.axhline(threshold, color="red", linestyle="--", label=f"Threshold ({threshold:.1f} N)")
        if contact_detected:
            ax.axvline(sample_times[-1], color="black", linestyle=":", label="Contact stop")
    ax.set_xlabel("Time from probe start (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title(f"Probe force over time: {label} ({'contact' if contact_detected else 'no contact'})")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    try:
        fig.savefig(output_path, dpi=160, bbox_inches="tight")
    finally:
        plt.close(fig)

    return output_path


def main(args=None):
    rclpy.init(args=args)
    node = DsrRg2MultiProbingNode()

    homej = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    base_ori = [0.0, 180.0, 0.0]
    z_offset_mm = -90.0

    # 물체 탐지 그리퍼 Z축 높이
    probe_z_boost_mm = 6.0
    tcp_probe_z_mm = (node.matrix_size[2] / 2.0) * 1000.0 + probe_z_boost_mm
    flange_probe_z_mm = tcp_probe_z_mm + node.gripper_length_mm + z_offset_mm
    safe_z_mm = flange_probe_z_mm + 45.0
    probe_plot_index = 0
    force_plot_dir = Path.cwd() / "tactile_force_logs"

    def save_probe_plot(label, sample_times, force_samples, contact_force_samples,
                        threshold, contact_detected):
        nonlocal probe_plot_index
        probe_plot_index += 1
        plot_path = save_force_time_plot(
            sample_times,
            force_samples,
            contact_force_samples,
            threshold,
            label,
            probe_plot_index,
            contact_detected,
            force_plot_dir,
        )
        node.get_logger().info(f"[힘-시간 그래프 저장] {plot_path}")

    def probe_direction(start_mm, dir_vec, face_label="", max_dist_mm=150.0, probe_vel=3.0, is_fine=False):
        start_xy = np.array(start_mm, dtype=float)
        d_xy = np.array(dir_vec, dtype=float)
        d_xy /= np.linalg.norm(d_xy)
        target_xy = start_xy + d_xy * max_dist_mm

        # [방법 A 핵심] 3D 대각선 진입 Z 높이 설정
        # 시작: 기준 탐지 높이보다 10mm 위에서 출발 -> 목표: 8mm 아래를 향해 대각선 하강
        z_drop_start_offset = 6.0
        z_drop_end_offset = -6.0

        start_flange_z = flange_probe_z_mm + z_drop_start_offset
        # 바닥 충돌 방지를 위해 최소 플랜지 Z 높이 제한 (TCP 기준 최소 3mm 이상 유지)
        min_safe_flange_z = 3.0 + node.gripper_length_mm + z_offset_mm
        target_flange_z = max(min_safe_flange_z, flange_probe_z_mm + z_drop_end_offset)

        # 3D 시작점, 목표점 및 3D 단위 방향 벡터 계산
        start_xyz = np.array([start_xy[0], start_xy[1], start_flange_z])
        target_xyz = np.array([target_xy[0], target_xy[1], target_flange_z])
        d_3d = target_xyz - start_xyz
        d_3d /= np.linalg.norm(d_3d)

        # 1. 안전 높이 이동 후 대각선 시작점(높은 위치)으로 하강
        node.send_movel([start_xy[0], start_xy[1], safe_z_mm] + base_ori, vel=100, acc=100, sync_type=0)
        node.send_movel([start_xy[0], start_xy[1], start_flange_z] + base_ori, vel=60, acc=60, sync_type=0)
        time.sleep(0.2)
        node.calibrate_force_baseline()

        # 2. 대각선 아래 방향(target_xyz)으로 직선 이동 시작
        node.send_movel([target_xy[0], target_xy[1], target_flange_z] + base_ori, vel=probe_vel, acc=5, sync_type=1)

        start_tcp_z_m = (start_flange_z - node.gripper_length_mm - z_offset_mm) / 1000.0
        path_m = [[start_xy[0]/1000.0, start_xy[1]/1000.0, start_tcp_z_m]]
        timeout_sec = (np.linalg.norm(target_xyz - start_xyz) / probe_vel) + 3.0
        t_start = time.time()
        consecutive_hits = 0
        sample_times = []
        force_samples = []
        contact_force_samples = []

        while rclpy.ok() and (time.time() - t_start < timeout_sec):
            curr_tcp = node.get_current_tcp_pos()
            curr_tcp_z_mm = float(curr_tcp[2]) - node.gripper_length_mm - z_offset_mm
            curr_wrench = node.get_current_wrench()
            diff_force = curr_wrench[:3] - node.baseline_wrench[:3]

            # [방법 A 핵심] XY 수평력 + Z축 수직 반발력을 모두 합산한 3D 대항력 계산
            opposing_force = float(-np.dot(diff_force[:3], d_3d))
            sample_times.append(time.time() - t_start)
            force_samples.append(diff_force.copy())
            contact_force_samples.append(opposing_force)

            path_m.append([curr_tcp[0]/1000.0, curr_tcp[1]/1000.0, curr_tcp_z_mm/1000.0])

            if (time.time() - t_start) > 0.25 and opposing_force >= node.force_threshold_xy:
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            if consecutive_hits >= 3:
                node.stop_robot_immediately()

                stopped_tcp = node.get_current_tcp_pos()
                contact_xy = stopped_tcp[:2]
                stopped_tcp_z_mm = float(stopped_tcp[2]) - node.gripper_length_mm - z_offset_mm
                contact_xyz_mm = [float(contact_xy[0]), float(contact_xy[1]), stopped_tcp_z_mm]

                force_info = {
                    "label": face_label,
                    "force_mag": opposing_force,
                    "fx": float(diff_force[0]),
                    "fy": float(diff_force[1]),
                    "fz": float(diff_force[2]),
                    "xy": contact_xy.copy(),
                    "xyz_mm": contact_xyz_mm,
                }

                node.objects[node.active_obj_idx]["remembered_contacts"].append(force_info)

                node.get_logger().info(
                    f"[{face_label} 대각선 접촉 정지] 3D 반력: {opposing_force:.2f} N (Fz: {diff_force[2]:.2f} N) | "
                    f"접촉 좌표: ({contact_xyz_mm[0]:.2f}, {contact_xyz_mm[1]:.2f}, {contact_xyz_mm[2]:.2f}) mm"
                )

                node.contact_points.append([contact_xy[0]/1000.0, contact_xy[1]/1000.0, stopped_tcp_z_mm/1000.0])
                if is_fine:
                    node.fine_paths.append(path_m)
                else:
                    node.coarse_paths.append(path_m)
                node.publish_visuals()
                save_probe_plot(
                    face_label,
                    sample_times,
                    force_samples,
                    contact_force_samples,
                    node.force_threshold_xy,
                    contact_detected=True,
                )

                # 후퇴 시 대각선 진입 방향의 정반대(뒤쪽+위쪽)로 빠져나와 물체를 끌지 않도록 함
                retract_xyz = stopped_tcp[:3] - d_3d * 15.0
                node.send_movel([retract_xyz[0], retract_xyz[1], retract_xyz[2]] + base_ori, vel=50, acc=50, sync_type=0)
                node.send_movel([retract_xyz[0], retract_xyz[1], safe_z_mm] + base_ori, vel=100, acc=100, sync_type=0)
                return contact_xy, force_info

            if np.linalg.norm(curr_tcp[:2] - target_xy) < 1.5:
                break

            time.sleep(0.02)

        node.send_movel([target_xy[0], target_xy[1], safe_z_mm] + base_ori, vel=100, acc=100, sync_type=0)
        save_probe_plot(
            face_label,
            sample_times,
            force_samples,
            contact_force_samples,
            node.force_threshold_xy,
            contact_detected=False,
        )
        return None, None

    def probe_top_z(xy_mm, ori_deg, start_tcp_z_mm=45.0, min_tcp_z_mm=10.0, probe_vel=3.0):
        start_flange_z = float(start_tcp_z_mm) + node.gripper_length_mm + z_offset_mm
        min_flange_z = float(min_tcp_z_mm) + node.gripper_length_mm + z_offset_mm

        node.send_movel([xy_mm[0], xy_mm[1], start_flange_z] + ori_deg, vel=80, acc=80, sync_type=0)
        time.sleep(0.2)
        node.calibrate_force_baseline()

        node.send_movel([xy_mm[0], xy_mm[1], min_flange_z] + ori_deg, vel=probe_vel, acc=5, sync_type=1)

        path_m = [[xy_mm[0]/1000.0, xy_mm[1]/1000.0, start_tcp_z_mm/1000.0]]
        timeout_sec = (abs(start_tcp_z_mm - min_tcp_z_mm) / probe_vel) + 3.0
        t_start = time.time()
        consecutive_hits = 0
        sample_times = []
        force_samples = []
        contact_force_samples = []

        while rclpy.ok() and (time.time() - t_start < timeout_sec):
            curr_tcp = node.get_current_tcp_pos()
            curr_tcp_z = float(curr_tcp[2]) - node.gripper_length_mm - z_offset_mm
            curr_wrench = node.get_current_wrench()
            diff_force = curr_wrench[:3] - node.baseline_wrench[:3]
            upward_force_z = float(max(0.0, diff_force[2]))
            sample_times.append(time.time() - t_start)
            force_samples.append(diff_force.copy())
            contact_force_samples.append(upward_force_z)

            path_m.append([xy_mm[0]/1000.0, xy_mm[1]/1000.0, curr_tcp_z/1000.0])

            if (time.time() - t_start) > 0.25 and upward_force_z >= node.force_threshold_z:
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            if consecutive_hits >= 3:
                node.stop_robot_immediately()

                measured_flange_z = float(node.get_current_tcp_pos()[2])
                measured_tcp_z = measured_flange_z - node.gripper_length_mm - z_offset_mm

                top_contact_info = {
                    "label": "상단 Z면",
                    "force_mag": upward_force_z,
                    "fx": float(diff_force[0]),
                    "fy": float(diff_force[1]),
                    "fz": float(diff_force[2]),
                    "xy": np.array([xy_mm[0], xy_mm[1]]),
                    "xyz_mm": [float(xy_mm[0]), float(xy_mm[1]), float(measured_tcp_z)],
                }
                node.objects[node.active_obj_idx]["remembered_contacts"].append(top_contact_info)

                node.get_logger().info(
                    f"[상단 Z축 접촉 정지] 반발력: {upward_force_z:.2f} N (임계: {node.force_threshold_z} N) | "
                    f"기억된 상단 접촉 좌표: ({xy_mm[0]:.2f}, {xy_mm[1]:.2f}, {measured_tcp_z:.2f}) mm"
                )
                node.contact_points.append([xy_mm[0]/1000.0, xy_mm[1]/1000.0, measured_tcp_z/1000.0])
                node.fine_paths.append(path_m)
                node.publish_visuals()

                node.send_movel([xy_mm[0], xy_mm[1], safe_z_mm] + ori_deg, vel=90, acc=90, sync_type=0)
                save_probe_plot(
                    "상단 Z면",
                    sample_times,
                    force_samples,
                    contact_force_samples,
                    node.force_threshold_z,
                    contact_detected=True,
                )
                return measured_tcp_z

            if abs(curr_tcp[2] - min_flange_z) < 1.0:
                break

            time.sleep(0.02)

        node.send_movel([xy_mm[0], xy_mm[1], safe_z_mm] + ori_deg, vel=90, acc=90, sync_type=0)
        save_probe_plot(
            "상단 Z면",
            sample_times,
            force_samples,
            contact_force_samples,
            node.force_threshold_z,
            contact_detected=False,
        )
        return None

    try:
        # 1. 로봇 이동 전에 먼저 그리퍼를 펄스 명령으로 확실히 닫고 1.5초 대기
        node.grip_for_probing(wait_sec=1.5)
        node.send_movej(homej, vel=60, acc=60)

        for idx, obj in enumerate(node.objects):
            node.active_obj_idx = idx
            node.reset_path_markers()
            obj["remembered_contacts"] = []
            zx, zy = obj["zone_center_mm"]

            # 탐지 시작 전 한 번 더 닫힘 펄스 확인 후 탐지 중에는 절대 열지 않음
            node.grip_for_probing(wait_sec=1.0)

            node.get_logger().info("==================================================")
            node.get_logger().info(
                f"[{idx+1}번 물체 주변 탐색 시작] 입력된 대략적 중심: ({zx:.2f}, {zy:.2f}) mm | "
                f"탐지 TCP Z 높이: {tcp_probe_z_mm:.1f} mm | 그리퍼: 닫힘 고정 | 정지 임계 힘: {node.force_threshold_xy} N"
            )
            node.get_logger().info("==================================================")

            # 1단계: 입력된 대략적 중심(zx, zy) 주변 4방향 탐색
            search_radius = 85.0
            p_south, _ = probe_direction([zx - search_radius, zy], [+1,  0], face_label="1단계 -X면", max_dist_mm=150.0)
            p_north, _ = probe_direction([zx + search_radius, zy], [-1,  0], face_label="1단계 +X면", max_dist_mm=150.0)
            p_east,  _ = probe_direction([zx, zy - search_radius], [ 0, +1], face_label="1단계 -Y면", max_dist_mm=150.0)
            p_west,  _ = probe_direction([zx, zy + search_radius], [ 0, -1], face_label="1단계 +Y면", max_dist_mm=150.0)

            if any(p is None for p in [p_south, p_north, p_east, p_west]):
                node.get_logger().error(f"1단계 탐색 실패! ({node.force_threshold_xy}N 이상 접촉 미감지)")
                continue

            rough_cx = (p_south[0] + p_north[0]) / 2.0
            rough_cy = (p_east[1] + p_west[1]) / 2.0

            # 2단계: 비틀림각(Yaw) 추정
            offset_mm = 10.0
            p_s1, _ = probe_direction([rough_cx - 60.0, rough_cy - offset_mm], [+1, 0], face_label="2단계 각도점1", max_dist_mm=100.0)
            p_s2, _ = probe_direction([rough_cx - 60.0, rough_cy + offset_mm], [+1, 0], face_label="2단계 각도점2", max_dist_mm=100.0)

            est_yaw_rad = math.atan2(-(p_s2[0] - p_s1[0]), (p_s2[1] - p_s1[1])) if (p_s1 is not None and p_s2 is not None) else 0.0
            est_yaw_deg = math.degrees(est_yaw_rad)

            # 3단계: 로컬 축 기준 4면 정밀 측정
            u_x = np.array([math.cos(est_yaw_rad), math.sin(est_yaw_rad)])
            u_y = np.array([-math.sin(est_yaw_rad), math.cos(est_yaw_rad)])
            rough_center = np.array([rough_cx, rough_cy])
            standoff = 60.0

            p_lx_neg, f_lx_neg = probe_direction(rough_center - u_x * standoff, +u_x, face_label="로컬X축 -면", max_dist_mm=100.0, is_fine=True)
            p_lx_pos, f_lx_pos = probe_direction(rough_center + u_x * standoff, -u_x, face_label="로컬X축 +면", max_dist_mm=100.0, is_fine=True)
            p_ly_neg, f_ly_neg = probe_direction(rough_center - u_y * standoff, +u_y, face_label="로컬Y축 -면", max_dist_mm=100.0, is_fine=True)
            p_ly_pos, f_ly_pos = probe_direction(rough_center + u_y * standoff, -u_y, face_label="로컬Y축 +면", max_dist_mm=100.0, is_fine=True)

            if any(p is None for p in [p_lx_neg, p_lx_pos, p_ly_neg, p_ly_pos]):
                node.get_logger().error("3단계 정밀 탐색 실패!")
                continue

            mid_x_vec = (p_lx_neg + p_lx_pos) / 2.0
            mid_y_vec = (p_ly_neg + p_ly_pos) / 2.0
            final_center = rough_center + np.dot(mid_x_vec - rough_center, u_x) * u_x + np.dot(mid_y_vec - rough_center, u_y) * u_y

            span_local_x_mm = float(np.linalg.norm(p_lx_pos - p_lx_neg))
            span_local_y_mm = float(np.linalg.norm(p_ly_pos - p_ly_neg))

            if span_local_x_mm <= span_local_y_mm:
                width_axis_name = "로컬 X축"
                length_axis_name = "로컬 Y축"
                width_p1, width_p2 = p_lx_neg, p_lx_pos
                width_f1, width_f2 = f_lx_neg, f_lx_pos
                length_p1, length_p2 = p_ly_neg, p_ly_pos
                length_f1, length_f2 = f_ly_neg, f_ly_pos
                width_span_mm = span_local_x_mm
                length_span_mm = span_local_y_mm
                width_center_xy = mid_x_vec
                length_center_xy = mid_y_vec
                long_axis_yaw_deg = normalize_deg(est_yaw_deg + 90.0)
                grasp_yaw_deg = normalize_deg(-est_yaw_deg+ 90.0)
            else:
                width_axis_name = "로컬 Y축"
                length_axis_name = "로컬 X축"
                width_p1, width_p2 = p_ly_neg, p_ly_pos
                width_f1, width_f2 = f_ly_neg, f_ly_pos
                length_p1, length_p2 = p_lx_neg, p_lx_pos
                length_f1, length_f2 = f_lx_neg, f_lx_pos
                width_span_mm = span_local_y_mm
                length_span_mm = span_local_x_mm
                width_center_xy = mid_y_vec
                length_center_xy = mid_x_vec
                long_axis_yaw_deg = normalize_deg(est_yaw_deg)
                grasp_yaw_deg = normalize_deg(-est_yaw_deg)

            # 4단계: 상단 Z축 터치 (그리퍼 닫힌 상태 유지)
            grasp_ori = [0.0, 180.0, grasp_yaw_deg]
            est_top_z_mm = probe_top_z(final_center, grasp_ori, start_tcp_z_mm=45.0, min_tcp_z_mm=2.0)
            if est_top_z_mm is None:
                node.get_logger().warn(f"상단 Z축 {node.force_threshold_z}N 접촉 미감지 -> 기본 상단 높이(20.0mm) 사용")
                est_top_z_mm = node.matrix_size[2] * 1000.0

            remembered_pose = [
                float(final_center[0]),
                float(final_center[1]),
                float(est_top_z_mm),
                0.0,
                180.0,
                float(grasp_yaw_deg),
            ]
            obj["remembered_pose"] = remembered_pose
            node.remembered_object_pose = remembered_pose

            obj["est_center"] = np.array([final_center[0]/1000.0, final_center[1]/1000.0, est_top_z_mm/2000.0])
            obj["est_yaw"] = math.radians(long_axis_yaw_deg)
            node.publish_visuals()

            plot_path = Path.cwd() / f"tactile_top_view_object_{obj['id']}.png"
            save_top_view_plot(
                obj["remembered_contacts"],
                final_center,
                length_mm=node.matrix_size[0] * 1000.0,
                width_mm=node.matrix_size[1] * 1000.0,
                long_axis_yaw_deg=long_axis_yaw_deg,
                output_path=plot_path,
            )
            node.get_logger().info(f"[상단 자세 추정 이미지 저장] {plot_path}")

            node.get_logger().info("==================================================")
            node.get_logger().info("[가로(41mm) / 세로(64mm) 위치 및 접촉 힘 추정 결과]")
            node.get_logger().info(
                f"  1) 가로면 (짧은 변, 41mm 기준) -> [{width_axis_name}] 판별\n"
                f"     - 가로 양 끝 접촉 좌표: ({width_p1[0]:.2f}, {width_p1[1]:.2f}) mm  <-->  ({width_p2[0]:.2f}, {width_p2[1]:.2f}) mm\n"
                f"     - 가로 양 면 측정 힘  : {width_f1['force_mag']:.2f} N / {width_f2['force_mag']:.2f} N (접촉 간격: {width_span_mm:.1f} mm)\n"
                f"     - 가로 축 중앙 좌표   : ({width_center_xy[0]:.2f}, {width_center_xy[1]:.2f}) mm"
            )
            node.get_logger().info(
                f"  2) 세로면 (긴 변, 64mm 기준)   -> [{length_axis_name}] 판별\n"
                f"     - 세로 양 끝 접촉 좌표: ({length_p1[0]:.2f}, {length_p1[1]:.2f}) mm  <-->  ({length_p2[0]:.2f}, {length_p2[1]:.2f}) mm\n"
                f"     - 세로 양 면 측정 힘  : {length_f1['force_mag']:.2f} N / {length_f2['force_mag']:.2f} N (접촉 간격: {length_span_mm:.1f} mm)\n"
                f"     - 세로 축 중앙 좌표   : ({length_center_xy[0]:.2f}, {length_center_xy[1]:.2f}) mm"
            )
            node.get_logger().info(
                f"  ★ 최종 추정된 물체 위치 좌표 (X, Y, 상단 Z, Rx, Ry, Rz): "
                f"[{remembered_pose[0]:.2f}, {remembered_pose[1]:.2f}, {remembered_pose[2]:.2f}, "
                f"{remembered_pose[3]:.1f}, {remembered_pose[4]:.1f}, {remembered_pose[5]:.1f}]"
            )
            node.get_logger().info("==================================================")

            # ------------------------------------------------------------------
            # 5단계: 최종 추정된 X, Y 중앙 좌표 상단에서 그리퍼 완전 개방 -> 하강 -> 완전 닫기 -> 상승
            # ------------------------------------------------------------------
            # 5-1. 최종 추정된 X, Y 중앙 좌표의 Z축 상단(safe_z_mm)으로 이동
            node.get_logger().info(
                f"[상단 위치 이동] 최종 추정 X, Y 중앙 좌표 ({remembered_pose[0]:.2f}, {remembered_pose[1]:.2f}) mm 상단(플랜지 Z: {safe_z_mm:.2f} mm)으로 이동"
            )
            node.send_movel([remembered_pose[0], remembered_pose[1], safe_z_mm] + grasp_ori, vel=80, acc=80, sync_type=0)

            # 5-2. Z축 상단에서 그리퍼를 완전히 벌림
            node.release_wide(wait_sec=1.5)

            # 5-3. 물체 위치(물체 중심 높이: 상단 Z에서 물체 높이 절반 하강)로 수직 하강
            grasp_descent_mm = (node.matrix_size[2] * 1000.0) / 2.0 + 8.0# 20mm 높이의 절반인 10.0mm 하강
            grasp_tcp_z_mm = max(2.0, remembered_pose[2] - grasp_descent_mm)
            grasp_flange_z_mm = grasp_tcp_z_mm + node.gripper_length_mm + z_offset_mm

            node.get_logger().info(
                f"[물체 위치 하강] 추정 상단 Z({remembered_pose[2]:.2f} mm)에서 {grasp_descent_mm:.1f} mm 하강 -> "
                f"목표 TCP Z: {grasp_tcp_z_mm:.2f} mm (플랜지 Z: {grasp_flange_z_mm:.2f} mm)"
            )
            node.send_movel([remembered_pose[0], remembered_pose[1], grasp_flange_z_mm] + grasp_ori, vel=25, acc=25, sync_type=0)
            time.sleep(0.3)

            # 5-4. 물체 위치에 도달 후 그리퍼를 완전히 닫음
            node.close_gripper_full_for_grasp(wait_sec=1.8)

            obj["state"] = "grasped"
            node.publish_visuals()

            # 5-5. 그리퍼가 완전히 닫힌 상태로 수직 상승(들어올리기)
            lift_z_mm = safe_z_mm + 40.0
            node.get_logger().info(f"[수직 상승 (Lift)] 물체를 잡은 상태로 목표 플랜지 Z: {lift_z_mm:.2f} mm 까지 들어올립니다.")
            node.send_movel([remembered_pose[0], remembered_pose[1], lift_z_mm] + grasp_ori, vel=45, acc=45, sync_type=0)

        node.get_logger().info("=== 최종 추정 중앙 좌표 파지(완전 개방 -> 하강 -> 완전 닫기 -> 상승) 완료 ===")
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
