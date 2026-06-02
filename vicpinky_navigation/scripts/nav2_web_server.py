#!/usr/bin/env python3
import threading
import time
import math
import os

from flask import Flask, jsonify, request, send_from_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap  # global/local costmap

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# TF2
from tf2_ros import Buffer, TransformListener

# SLAM Toolbox services
from slam_toolbox.srv import SaveMap, Reset
from std_msgs.msg import String


############################################################
# Flask 설정
############################################################

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(
    __name__,
    static_folder=BASE_DIR,   # 같은 폴더의 index.html 서빙
    static_url_path=""
)

ros_node = None   # 전역 ROS 노드 포인터


############################################################
# 유틸: Quaternion → yaw
############################################################
def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


############################################################
# ROS2 노드 (Flask 브리지)
############################################################
class Nav2WebBridge(Node):
    def __init__(self):
        super().__init__("nav2_web_bridge_tf")
        
        self.declare_parameter("ip", "192.168.4.1")
        self.declare_parameter("port", 8080)

        # ROS 데이터
        self.map_msg = None
        self.path_msg = None
        self.local_costmap_msg = None
        self.global_costmap_msg = None

        # TF 기반 pose (x,y,yaw)
        self.tf_pose = None  # (x, y, yaw)

        self.lock = threading.Lock()

        # ---- map: TRANSIENT_LOCAL QoS (latched) ----
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "map",              # 필요시 실제 토픽 이름으로 수정
            self.map_callback,
            map_qos,
        )

        # ---- path: 기본 QoS ----
        self.create_subscription(
            Path,
            "plan",
            self.path_callback,
            10,
        )

        # ---- local costmap ----
        self.local_costmap_seen = False
        self.create_subscription(
            Costmap,
            "local_costmap/costmap",
            self.local_costmap_callback,
            10,
        )
        self.create_subscription(
            Costmap,
            "local_costmap/costmap_raw",
            self.local_costmap_callback,
            10,
        )

        # ---- global costmap ----
        self.global_costmap_seen = False
        self.create_subscription(
            Costmap,
            "global_costmap/costmap",
            self.global_costmap_callback,
            10,
        )
        self.create_subscription(
            Costmap,
            "global_costmap/costmap_raw",
            self.global_costmap_callback,
            10,
        )

        # ---- TF2: map -> base_link / odom ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # 주기적으로 TF에서 pose 업데이트
        self.create_timer(0.1, self.update_pose_from_tf)

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Nav2 주행 상태 추적용
        self._goal_handle = None      # 현재 활성 goal handle (취소에 사용)
        self._is_navigating = False   # Nav2가 현재 주행 중인지 여부

        # ---- SLAM Toolbox 서비스 클라이언트 ----
        self.save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.reset_client = self.create_client(Reset, "/slam_toolbox/reset")

        self.get_logger().info("Nav2WebBridge (TF-based + SLAM) started.")

    # ---------------- 콜백 ----------------
    def map_callback(self, msg):
        with self.lock:
            self.map_msg = msg

    def path_callback(self, msg):
        with self.lock:
            self.path_msg = msg

    def local_costmap_callback(self, msg):
        with self.lock:
            self.local_costmap_msg = msg
        if not self.local_costmap_seen:
            self.local_costmap_seen = True
            self.get_logger().info(
                f"Received first LOCAL costmap: "
                f"size=({msg.metadata.size_x}, {msg.metadata.size_y}), "
                f"res={msg.metadata.resolution}"
            )

    def global_costmap_callback(self, msg):
        with self.lock:
            self.global_costmap_msg = msg
        if not self.global_costmap_seen:
            self.global_costmap_seen = True
            self.get_logger().info(
                f"Received first GLOBAL costmap: "
                f"size=({msg.metadata.size_x}, {msg.metadata.size_y}), "
                f"res={msg.metadata.resolution}"
            )

    # ---------------- TF에서 pose 업데이트 ----------------
    def update_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
            t = trans.transform
            x = t.translation.x
            y = t.translation.y
            yaw = quat_to_yaw(t.rotation)

            with self.lock:
                self.tf_pose = (x, y, yaw)

        except Exception:
            pass

    # ---------------- 임의 프레임의 원점을 map 프레임으로 정밀 변환 ----------------
    def transform_origin_to_map(self, x, y, yaw, from_frame):
        if from_frame == "map" or not from_frame:
            return x, y, yaw
        try:
            # map 프레임 기준의 해당 프레임 위치 조회
            trans = self.tf_buffer.lookup_transform("map", from_frame, Time())
            t = trans.transform
            tx = t.translation.x
            ty = t.translation.y
            t_yaw = quat_to_yaw(t.rotation)

            # 회전 및 변환 행렬 적용 (2D 정밀 정렬)
            cos_t = math.cos(t_yaw)
            sin_t = math.sin(t_yaw)

            map_x = x * cos_t - y * sin_t + tx
            map_y = x * sin_t + y * cos_t + ty
            map_yaw = yaw + t_yaw
            return map_x, map_y, map_yaw
        except Exception:
            # TF를 아직 가져오지 못한 경우 기본값 그대로 사용
            return x, y, yaw

    # ---------------- JSON 스냅샷 ----------------
    def get_state_snapshot(self):
        with self.lock:
            map_msg = self.map_msg
            path_msg = self.path_msg
            local_costmap_msg = self.local_costmap_msg
            global_costmap_msg = self.global_costmap_msg
            tf_pose = self.tf_pose
            is_navigating = self._is_navigating

        # map
        map_json = None
        if map_msg is not None:
            info = map_msg.info
            map_json = {
                "width": info.width,
                "height": info.height,
                "resolution": info.resolution,
                "origin": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                    "yaw": quat_to_yaw(info.origin.orientation)
                },
                "data": list(map_msg.data),
            }

        # pose
        pose_json = None
        if tf_pose is not None:
            x, y, yaw = tf_pose
            pose_json = {
                "x": x,
                "y": y,
                "yaw": yaw,
            }

        # path
        path_json = []
        if path_msg is not None:
            for ps in path_msg.poses:
                path_json.append({
                    "x": ps.pose.position.x,
                    "y": ps.pose.position.y,
                })

        # local costmap (TF 정렬 적용)
        local_costmap_json = None
        if local_costmap_msg is not None and len(local_costmap_msg.data) > 0:
            meta = local_costmap_msg.metadata
            frame_id = local_costmap_msg.header.frame_id
            
            # 원래의 원점 좌표
            lx = meta.origin.position.x
            ly = meta.origin.position.y
            lyaw = quat_to_yaw(meta.origin.orientation)
            
            # odom -> map 좌표 정밀 변환 수행
            map_lx, map_ly, map_lyaw = self.transform_origin_to_map(lx, ly, lyaw, frame_id)

            local_costmap_json = {
                "width": meta.size_x,
                "height": meta.size_y,
                "resolution": meta.resolution,
                "origin": {
                    "x": map_lx,
                    "y": map_ly,
                    "yaw": map_lyaw,
                },
                "data": list(local_costmap_msg.data),
            }

        # global costmap (TF 정렬 적용)
        global_costmap_json = None
        if global_costmap_msg is not None and len(global_costmap_msg.data) > 0:
            meta = global_costmap_msg.metadata
            frame_id = global_costmap_msg.header.frame_id
            
            gx = meta.origin.position.x
            gy = meta.origin.position.y
            gyaw = quat_to_yaw(meta.origin.orientation)
            
            # 혹시라도 map 프레임이 아닐 경우를 대비해 변환 지원
            map_gx, map_gy, map_gyaw = self.transform_origin_to_map(gx, gy, gyaw, frame_id)

            global_costmap_json = {
                "width": meta.size_x,
                "height": meta.size_y,
                "resolution": meta.resolution,
                "origin": {
                    "x": map_gx,
                    "y": map_gy,
                    "yaw": map_gyaw,
                },
                "data": list(global_costmap_msg.data),
            }

        return {
            "map": map_json,
            "pose": pose_json,
            "path": path_json,
            "local_costmap": local_costmap_json,
            "global_costmap": global_costmap_json,
            "navigating": is_navigating,
        }

    # ---------------- Goal 전송 ----------------
    def send_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("navigate_to_pose Action Server not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"[WEB] send goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        with self.lock:
            self._is_navigating = True
        return True

    # ---------------- Nav2 주행 상태/취소 ----------------
    def _goal_response_cb(self, future):
        """goal 수락 여부 확인 후, 활성 handle 저장 및 결과 콜백 등록."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[WEB] Goal rejected by Nav2.")
            with self.lock:
                self._is_navigating = False
                self._goal_handle = None
            return

        with self.lock:
            self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        """주행 종료(성공/실패/취소) 시 상태 초기화."""
        with self.lock:
            self._is_navigating = False
            self._goal_handle = None
        self.get_logger().info("[WEB] Navigation finished.")

    def is_navigating(self) -> bool:
        with self.lock:
            return self._is_navigating

    def cancel_goal(self) -> bool:
        """현재 Nav2 주행을 취소(정지)."""
        with self.lock:
            goal_handle = self._goal_handle

        if goal_handle is None:
            self.get_logger().info("[WEB] No active goal to cancel.")
            return True

        goal_handle.cancel_goal_async()
        self.get_logger().info("[WEB] Requested goal cancel (stop).")
        return True

    # ---------------- SLAM Toolbox 제어 ----------------
    def slam_reset(self) -> bool:
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/reset service not available.")
            return False

        req = Reset.Request()
        self.reset_client.call_async(req)
        self.get_logger().info("[WEB] Requested SLAM reset.")
        return True

    def slam_save_map(self, name: str) -> bool:
        if not self.save_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/save_map service not available.")
            return False

        req = SaveMap.Request()
        req.name = String(data=name)

        self.save_map_client.call_async(req)
        self.get_logger().info(f"[WEB] Requested SLAM save_map: name='{name}'")
        return True


############################################################
# Flask 라우트
############################################################

@app.route("/")
def serve_index():
    return send_from_directory(BASE_DIR, "index.html")


@app.route("/api/state")
def api_state():
    global ros_node
    if ros_node is None:
        return jsonify({"error": "ROS node not started"}), 500

    return jsonify(ros_node.get_state_snapshot())


@app.route("/api/goal", methods=["POST"])
def api_goal():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    data = request.get_json()
    x = float(data["x"])
    y = float(data["y"])
    yaw = float(data.get("yaw", 0.0))

    ok = ros_node.send_goal(x, y, yaw)
    return jsonify({"success": ok})


@app.route("/api/nav/status")
def api_nav_status():
    global ros_node
    if ros_node is None:
        return jsonify({"error": "ROS node not started"}), 500

    return jsonify({"navigating": ros_node.is_navigating()})


@app.route("/api/nav/stop", methods=["POST"])
def api_nav_stop():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    ok = ros_node.cancel_goal()
    return jsonify({"success": ok})


@app.route("/api/slam/reset", methods=["POST"])
def api_slam_reset():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    ok = ros_node.slam_reset()
    return jsonify({"success": ok})


@app.route("/api/slam/save_map", methods=["POST"])
def api_slam_save_map():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    data = request.get_json() or {}
    name = data.get("name", "").strip()
    if not name:
        name = time.strftime("pinky_map_%Y%m%d_%H%M%S")

    ok = ros_node.slam_save_map(name)
    return jsonify({"success": ok, "name": name})


############################################################
# ROS 스레드
############################################################
def ros_spin_thread():
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


############################################################
# 메인 실행부
############################################################
if __name__ == "__main__":
    rclpy.init()    
    
    ros_node = Nav2WebBridge()

    ip_param = ros_node.get_parameter("ip").value
    port_param = ros_node.get_parameter("port").value

    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    time.sleep(1.0)

    print(f"Flask Web Server Running on http://{ip_param}:{port_param}")
    app.run(host=ip_param, port=int(port_param), debug=False)