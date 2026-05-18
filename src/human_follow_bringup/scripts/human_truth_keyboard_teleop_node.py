#!/usr/bin/env python3
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

from human_follow_msgs.msg import Target3D

try:
    import tkinter as tk
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("tkinter is required for human_truth manual panel") from exc


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def _phase_descriptor(label, visible, display_visible, truth_valid, phase_index):
    return "label=%s;visible=%d;display_visible=%d;truth_valid=%d;phase_index=%d" % (
        label,
        1 if visible else 0,
        1 if display_visible else 0,
        1 if truth_valid else 0,
        int(phase_index),
    )


class HumanTruthKeyboardTeleopNode:
    def __init__(self):
        self.truth_topic = rospy.get_param("~truth_topic", "/follow/sim/truth_target_body")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.nav_goal_topic = rospy.get_param("~nav_goal_topic", "/move_base_simple/goal")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.track_id = int(rospy.get_param("~track_id", 1))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.initial_x_m = float(rospy.get_param("~initial_x_m", 4.0))
        self.initial_y_m = float(rospy.get_param("~initial_y_m", 0.0))
        self.initial_z_m = float(rospy.get_param("~initial_z_m", -1.6))
        self.visible = bool(rospy.get_param("~visible", True))
        self.display_visible = bool(rospy.get_param("~display_visible", self.visible))
        self.display_visibility_controls_tracking = bool(
            rospy.get_param("~display_visibility_controls_tracking", False)
        )
        self.truth_valid = bool(rospy.get_param("~truth_valid", True))
        self.startup_focus_refresh_count = max(int(rospy.get_param("~startup_focus_refresh_count", 12)), 0)
        self.startup_focus_refresh_interval_ms = max(
            int(rospy.get_param("~startup_focus_refresh_interval_ms", 350)),
            50,
        )
        self.post_goal_focus_refresh_count = max(int(rospy.get_param("~post_goal_focus_refresh_count", 8)), 0)
        self.publish_default_target_on_startup = bool(
            rospy.get_param("~publish_default_target_on_startup", True)
        )
        self.clamp_goal_to_visible_envelope = bool(
            rospy.get_param("~clamp_goal_to_visible_envelope", False)
        )
        self.goal_min_forward_m = float(rospy.get_param("~goal_min_forward_m", 0.4))
        self.goal_max_forward_m = float(rospy.get_param("~goal_max_forward_m", 10.0))
        self.goal_max_lateral_abs_m = float(rospy.get_param("~goal_max_lateral_abs_m", 4.0))
        self.visibility_control_service = str(
            rospy.get_param("~visibility_control_service", "~set_visible")
        ).strip() or "~set_visible"

        if not os.environ.get("DISPLAY", "").strip():
            raise RuntimeError("DISPLAY is not set; manual panel UI needs X11")

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0
        self.vehicle_yaw_rad = 0.0
        self.vehicle_pose_ready = False

        self.world_position_x = None
        self.world_position_y = None
        self.world_position_z = None

        self.last_input_source = "waiting_for_odom"
        self.last_goal_frame = "-"
        self.last_goal_stamp = None
        self.last_status_note = "等待里程计"
        self.pending_focus_refresh_count = 0

        self.truth_publisher = rospy.Publisher(self.truth_topic, Target3D, queue_size=10)
        self.phase_publisher = rospy.Publisher(self.phase_topic, String, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.goal_subscriber = rospy.Subscriber(self.nav_goal_topic, PoseStamped, self._goal_callback, queue_size=20)
        self.visibility_service = rospy.Service(
            self.visibility_control_service,
            SetBool,
            self._handle_set_visible,
        )

        self.root = tk.Tk()
        self.root.title("Human Truth Manual Panel")
        self.root.geometry("620x290+60+60")
        self.root.configure(bg="#161a21")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<KeyPress>", self._on_key_press)

        title = tk.Label(
            self.root,
            text="Stage1 Sim Human Manual Panel",
            font=("DejaVu Sans", 16, "bold"),
            fg="#f8fafc",
            bg="#161a21",
        )
        title.pack(pady=(14, 8))

        visibility_help = (
            "shown/hidden（hidden=真实遮挡）"
            if self.display_visibility_controls_tracking
            else "只切换显示"
        )
        help_text = (
            "人在 RViz 中通过 2D Nav Goal 设位置\n"
            "只取落点位置，拖拽方向不会用于人体朝向\n"
            "Space 清空面板状态提示  R 复位到机前  V %s  Esc/Q 退出" % visibility_help
        )
        self.help_label = tk.Label(
            self.root,
            text=help_text,
            justify="left",
            font=("DejaVu Sans Mono", 11),
            fg="#cbd5e1",
            bg="#161a21",
        )
        self.help_label.pack(padx=16, pady=(0, 12), anchor="w")

        self.state_var = tk.StringVar()
        self.state_label = tk.Label(
            self.root,
            textvariable=self.state_var,
            justify="left",
            font=("DejaVu Sans Mono", 11),
            fg="#7dd3fc",
            bg="#161a21",
        )
        self.state_label.pack(padx=16, pady=(0, 10), anchor="w")

        footer = tk.Label(
            self.root,
            text="这个面板只负责 V/Space/R；点完 RViz 的 2D Nav Goal 后会自动把键盘焦点拉回这里",
            font=("DejaVu Sans", 10),
            fg="#fbbf24",
            bg="#161a21",
        )
        footer.pack(padx=16, pady=(0, 8), anchor="w")

        self._focus_window()
        self._schedule_focus_refresh(self.startup_focus_refresh_count)

        rospy.loginfo(
            "human_truth manual panel ready: truth=%s phase=%s odom=%s nav_goal=%s mode=rviz_nav_goal initial_offset=(%.2f, %.2f, %.2f)",
            self.truth_topic,
            self.phase_topic,
            self.odom_topic,
            self.nav_goal_topic,
            self.initial_x_m,
            self.initial_y_m,
            self.initial_z_m,
        )

    def _focus_window(self):
        self.root.lift()
        try:
            self.root.focus_force()
        except tk.TclError:
            return

    def _schedule_focus_refresh(self, remaining):
        if remaining <= 0:
            return
        self._focus_window()
        try:
            self.root.after(
                self.startup_focus_refresh_interval_ms,
                lambda remaining=remaining - 1: self._schedule_focus_refresh(remaining),
            )
        except tk.TclError:
            return

    def _request_focus_refresh(self, count_value):
        self.pending_focus_refresh_count = max(int(count_value), self.pending_focus_refresh_count)

    def _drain_focus_refresh_request(self):
        if self.pending_focus_refresh_count <= 0:
            return
        self._focus_window()
        self.pending_focus_refresh_count -= 1

    def _odom_callback(self, msg):
        self.vehicle_x = float(msg.pose.pose.position.x)
        self.vehicle_y = float(msg.pose.pose.position.y)
        self.vehicle_z = float(msg.pose.pose.position.z)
        self.vehicle_yaw_rad = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        was_ready = self.vehicle_pose_ready
        self.vehicle_pose_ready = True
        if not was_ready and self.world_position_x is None:
            if self.publish_default_target_on_startup:
                self._reset_target_to_default_pose()
                self.last_input_source = "startup_default"
                self.last_status_note = "已收到里程计，目标初始化到机前"
            else:
                self.last_input_source = "waiting_for_nav_goal"
                self.last_status_note = "已收到里程计，等待 RViz 2D Nav Goal"

    def _goal_callback(self, msg):
        frame_id = str(msg.header.frame_id or self.world_frame_id).strip() or self.world_frame_id
        self.last_goal_frame = frame_id
        self.last_goal_stamp = rospy.Time.now()

        if not self.vehicle_pose_ready:
            self.last_input_source = "goal_waiting_odom"
            self.last_status_note = "里程计未就绪，暂未接受 2D Nav Goal"
            rospy.logwarn_throttle(2.0, "manual panel ignoring nav goal until vehicle odom is ready")
            return

        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        if frame_id == self.world_frame_id:
            world_x = goal_x
            world_y = goal_y
        elif frame_id in (self.base_frame_id, self.frame_id):
            cos_yaw = math.cos(self.vehicle_yaw_rad)
            sin_yaw = math.sin(self.vehicle_yaw_rad)
            world_x = self.vehicle_x + cos_yaw * goal_x - sin_yaw * goal_y
            world_y = self.vehicle_y + sin_yaw * goal_x + cos_yaw * goal_y
        else:
            self.last_input_source = "goal_rejected"
            self.last_status_note = "不支持的 goal frame: %s" % frame_id
            rospy.logwarn_throttle(
                2.0,
                "manual panel rejected nav goal with unsupported frame '%s' (expected %s or %s)",
                frame_id,
                self.world_frame_id,
                self.base_frame_id,
            )
            return

        if self.world_position_z is None:
            self.world_position_z = self.vehicle_z + self.initial_z_m
        world_x, world_y, clamped = self._clamp_world_goal_to_visible_envelope(world_x, world_y)
        self.world_position_x = world_x
        self.world_position_y = world_y
        self.last_input_source = "rviz_goal"
        self.last_status_note = "已接受 2D Nav Goal"
        if clamped:
            self.last_status_note = (
                "2D Nav Goal 超出演示包线，已压回到前向[%.1f, %.1f]m、横向|y|<=%.1fm"
                % (
                    self.goal_min_forward_m,
                    self.goal_max_forward_m,
                    self.goal_max_lateral_abs_m,
                )
            )
        self._request_focus_refresh(self.post_goal_focus_refresh_count)
        rospy.loginfo(
            "manual panel accepted nav goal: frame=%s world=(%.2f, %.2f, %.2f) clamped=%s",
            frame_id,
            self.world_position_x,
            self.world_position_y,
            self.world_position_z,
            clamped,
        )

    def _on_close(self):
        rospy.signal_shutdown("human truth manual panel window closed")

    def _set_display_visible(self, visible_value, input_source, note_text):
        self.display_visible = bool(visible_value)
        self.last_input_source = input_source
        self.last_status_note = note_text
        rospy.loginfo(
            "manual panel set visibility: display=%s tracking=%s source=%s",
            "shown" if self.display_visible else "hidden",
            "visible" if self._tracking_visible() else "occluded",
            input_source,
        )

    def _handle_set_visible(self, request):
        target_visible = bool(request.data)
        self._set_display_visible(
            target_visible,
            "service_visibility",
            "服务切到 %s" % ("shown" if target_visible else "hidden"),
        )
        return SetBoolResponse(
            success=True,
            message="display=%s tracking=%s" % (
                "shown" if self.display_visible else "hidden",
                "visible" if self._tracking_visible() else "occluded",
            ),
        )

    def _clamp_world_goal_to_visible_envelope(self, world_x, world_y):
        if not self.clamp_goal_to_visible_envelope or not self.vehicle_pose_ready:
            return world_x, world_y, False

        dx_world = float(world_x) - self.vehicle_x
        dy_world = float(world_y) - self.vehicle_y
        cos_yaw = math.cos(self.vehicle_yaw_rad)
        sin_yaw = math.sin(self.vehicle_yaw_rad)
        body_x = cos_yaw * dx_world + sin_yaw * dy_world
        body_y = -sin_yaw * dx_world + cos_yaw * dy_world

        clamped_body_x = min(max(body_x, self.goal_min_forward_m), self.goal_max_forward_m)
        clamped_body_y = min(max(body_y, -self.goal_max_lateral_abs_m), self.goal_max_lateral_abs_m)
        clamped = (
            abs(clamped_body_x - body_x) > 1e-6
            or abs(clamped_body_y - body_y) > 1e-6
        )
        if not clamped:
            return world_x, world_y, False

        clamped_world_x = self.vehicle_x + cos_yaw * clamped_body_x - sin_yaw * clamped_body_y
        clamped_world_y = self.vehicle_y + sin_yaw * clamped_body_x + cos_yaw * clamped_body_y
        return clamped_world_x, clamped_world_y, True

    def _normalize_key(self, key_name):
        return str(key_name or "").strip().lower()

    def _clear_panel_state(self):
        self.last_input_source = "panel_cleared"
        self.last_goal_stamp = None
        self.last_goal_frame = "-"
        self.last_status_note = "面板状态已清空，当前位置保持不变"
        rospy.loginfo("manual panel cleared status note")

    def _reset_target_to_default_pose(self):
        if not self.vehicle_pose_ready:
            return False
        cos_yaw = math.cos(self.vehicle_yaw_rad)
        sin_yaw = math.sin(self.vehicle_yaw_rad)
        self.world_position_x = self.vehicle_x + cos_yaw * self.initial_x_m - sin_yaw * self.initial_y_m
        self.world_position_y = self.vehicle_y + sin_yaw * self.initial_x_m + cos_yaw * self.initial_y_m
        self.world_position_z = self.vehicle_z + self.initial_z_m
        return True

    def _body_truth_from_world(self):
        if not self.vehicle_pose_ready:
            return None
        if self.world_position_x is None:
            if not self.publish_default_target_on_startup:
                return None
            if not self._reset_target_to_default_pose():
                return None
        dx_world = float(self.world_position_x) - self.vehicle_x
        dy_world = float(self.world_position_y) - self.vehicle_y
        dz_world = float(self.world_position_z) - self.vehicle_z
        cos_yaw = math.cos(self.vehicle_yaw_rad)
        sin_yaw = math.sin(self.vehicle_yaw_rad)
        body_x = cos_yaw * dx_world + sin_yaw * dy_world
        body_y = -sin_yaw * dx_world + cos_yaw * dy_world
        return body_x, body_y, dz_world

    def _tracking_visible(self):
        if self.display_visibility_controls_tracking:
            return bool(self.display_visible)
        return bool(self.visible)

    def _on_key_press(self, event):
        key_name = self._normalize_key(event.keysym)
        if key_name in ("space", "x"):
            self._clear_panel_state()
            return
        if key_name == "r":
            if self._reset_target_to_default_pose():
                self.last_input_source = "reset_default"
                self.last_goal_stamp = rospy.Time.now()
                self.last_goal_frame = self.world_frame_id
                self.last_status_note = "目标已复位到机前默认位置"
                rospy.loginfo(
                    "manual panel reset target to default world=(%.2f, %.2f, %.2f)",
                    self.world_position_x,
                    self.world_position_y,
                    self.world_position_z,
                )
            else:
                self.last_input_source = "reset_waiting_odom"
                self.last_status_note = "里程计未就绪，暂不能复位"
                rospy.logwarn("manual panel reset requested before odom was ready")
            return
        if key_name == "v":
            self._set_display_visible(
                not self.display_visible,
                "toggle_visibility",
                "显示状态切到 %s" % ("shown" if not self.display_visible else "hidden"),
            )
            return
        if key_name in ("escape", "q"):
            self._on_close()

    def _update_state_text(self):
        if not self.vehicle_pose_ready:
            self.state_var.set(
                "input=%s  display=%s  tracking=%s\n等待 /follow/sim/vehicle_odom ...\ngoal_topic=%s  world_frame=%s\nnote=%s"
                % (
                    self.last_input_source,
                    "shown" if self.display_visible else "hidden",
                    "visible" if self._tracking_visible() else "occluded",
                    self.nav_goal_topic,
                    self.world_frame_id,
                    self.last_status_note,
                )
            )
            return

        body_truth = self._body_truth_from_world()
        if body_truth is None:
            self.state_var.set(
                "input=%s  display=%s  tracking=%s\n里程计已就绪，等待 2D Nav Goal ...\ngoal_topic=%s  world_frame=%s\nnote=%s"
                % (
                    self.last_input_source,
                    "shown" if self.display_visible else "hidden",
                    "visible" if self._tracking_visible() else "occluded",
                    self.nav_goal_topic,
                    self.world_frame_id,
                    self.last_status_note,
                )
            )
            return
        body_x, body_y, body_z = body_truth
        goal_age_text = "-"
        if self.last_goal_stamp is not None:
            goal_age_text = "%.1fs" % max(0.0, (rospy.Time.now() - self.last_goal_stamp).to_sec())

        self.state_var.set(
            "input=%s  display=%s  tracking=%s\nbody_x=%.2f m  body_y=%.2f m  body_z=%.2f m\nworld_x=%.2f m  world_y=%.2f m  goal_age=%s\nlast_goal_frame=%s  goal_topic=%s\nnote=%s"
            % (
                self.last_input_source,
                "shown" if self.display_visible else "hidden",
                "visible" if self._tracking_visible() else "occluded",
                body_x,
                body_y,
                body_z,
                self.world_position_x,
                self.world_position_y,
                goal_age_text,
                self.last_goal_frame,
                self.nav_goal_topic,
                self.last_status_note,
            )
        )

    def _publish_truth(self):
        body_truth = self._body_truth_from_world()
        if body_truth is None:
            return
        body_x, body_y, body_z = body_truth
        now = rospy.Time.now()

        truth_msg = Target3D()
        truth_msg.header.stamp = now
        truth_msg.header.frame_id = self.frame_id
        truth_msg.track_id = self.track_id
        truth_msg.frame_id = self.frame_id
        truth_msg.valid = bool(self.truth_valid)
        truth_msg.confidence = 1.0 if truth_msg.valid else 0.0
        truth_msg.support_points = 0
        truth_msg.position.x = float(body_x)
        truth_msg.position.y = float(body_y)
        truth_msg.position.z = float(body_z)

        phase_msg = String()
        phase_msg.data = _phase_descriptor(
            "manual_nav_goal",
            self._tracking_visible(),
            self.display_visible,
            self.truth_valid,
            0,
        )

        try:
            self.truth_publisher.publish(truth_msg)
            self.phase_publisher.publish(phase_msg)
        except rospy.ROSException:
            return

    def run(self):
        rate = rospy.Rate(max(self.publish_rate_hz, 1.0))
        while not rospy.is_shutdown():
            try:
                self.root.update_idletasks()
                self.root.update()
            except tk.TclError:
                rospy.signal_shutdown("human truth manual panel UI closed")
                break

            self._drain_focus_refresh_request()
            self._update_state_text()
            self._publish_truth()
            rate.sleep()

        try:
            self.root.destroy()
        except tk.TclError:
            pass


if __name__ == "__main__":
    rospy.init_node("human_truth_keyboard_teleop")
    node = HumanTruthKeyboardTeleopNode()
    node.run()
