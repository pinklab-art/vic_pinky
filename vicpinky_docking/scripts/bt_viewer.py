#!/usr/bin/env python3
"""도킹 BT 진행 보기 창.

/docking/behavior_tree_log 를 받아 BT 노드를 색으로 칠한다 (회색 대기 / 노랑 실행 중 / 초록 성공 / 빨강 실패).
DockRobot 노드 밑에는 도킹 서버 피드백을, 오른쪽 위에는 마커가 보이는지를 표시한다.

    ros2 run vicpinky_docking bt_viewer.py
    ros2 run vicpinky_docking bt_viewer.py --ros-args -p scale:=0.7
"""
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import DockRobot
from nav2_msgs.msg import BehaviorTreeLog
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node

W, H = 1200, 640
COLORS = {  # RGB
    'IDLE': (215, 215, 215), 'RUNNING': (255, 200, 40),
    'SUCCESS': (90, 190, 90), 'FAILURE': (230, 80, 70),
}
STATUS_KO = {'IDLE': '대기', 'RUNNING': '실행 중', 'SUCCESS': '성공', 'FAILURE': '실패'}
DOCK_STATE_KO = {0: '', 1: 'staging 이동', 2: '마커 인식', 3: '접근 제어', 4: '충전 대기', 5: '재시도'}
CONTROL_TAGS = {'Sequence', 'Fallback', 'ReactiveSequence', 'ReactiveFallback', 'PipelineSequence',
                'RecoveryNode', 'RoundRobin', 'Parallel', 'SequenceWithMemory'}


def korean_font(size):
    path = subprocess.run(['fc-match', '-f', '%{file}', ':lang=ko'],
                          capture_output=True, text=True).stdout.strip()
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


class TreeNode:
    def __init__(self, elem):
        self.tag = elem.tag
        self.name = elem.get('name', elem.tag)
        self.attrs = elem.attrib
        self.children = [TreeNode(c) for c in elem if isinstance(c.tag, str)]
        self.parent = None
        for c in self.children:
            c.parent = self
        self.status = 'IDLE'
        self.t_start = None
        self.t_end = None
        self.x = self.y = 0


def load_tree(path):
    root = ET.parse(path).getroot()
    main_id = root.get('main_tree_to_execute')
    bts = root.findall('BehaviorTree')
    bt = next((b for b in bts if b.get('ID') == main_id), bts[0])
    return TreeNode(list(bt)[0])


def leaves(node):
    return 1 if not node.children else sum(leaves(c) for c in node.children)


def layout(node, depth, x0, x1):
    """잎 노드 수만큼 폭을 나눠 주고(겹치지 않게), 세로로는 깊이마다 한 줄."""
    node.x = (x0 + x1) / 2
    node.y = 130 + depth * 160
    x = x0
    for c in node.children:
        w = (x1 - x0) * leaves(c) / leaves(node)
        layout(c, depth + 1, x, x + w)
        x += w


def walk(node):
    yield node
    for c in node.children:
        yield from walk(c)


class BtViewer(Node):
    def __init__(self):
        super().__init__('bt_viewer')
        default_xml = os.path.join(get_package_share_directory('vicpinky_docking'),
                                   'behavior_trees', 'dock_to_station.xml')
        xml = self.declare_parameter('bt_xml', default_xml).value
        log_topic = self.declare_parameter('log_topic', '/docking/behavior_tree_log').value
        action = self.declare_parameter('action', '/docking/navigate_to_pose').value
        self.scale = float(self.declare_parameter('scale', 1.0).value)
        self.root = load_tree(xml)
        layout(self.root, 0, 20, W - 20)
        self.by_name = {n.name: n for n in walk(self.root)}
        self.slot = (W - 40) / leaves(self.root)
        self.dock_state = 0
        self.dock_retries = 0
        self.last_det = 0.0
        self.det_z = None
        self.create_subscription(BehaviorTreeLog, log_topic, self.on_log, 50)
        self.create_subscription(DockRobot.Impl.FeedbackMessage, '/dock_robot/_action/feedback',
                                 self.on_dock_fb, 50)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_det, 10)
        # BT 실행기는 마지막 틱(끝난 순간)의 상태 변화를 로그로 내지 않는다 → 액션 결과로 마무리한다
        self.create_subscription(GoalStatusArray, f'{action}/_action/status', self.on_status, 10)
        self.f_big, self.f_mid, self.f_small = korean_font(30), korean_font(22), korean_font(17)
        self.get_logger().info(f'BT: {xml}  log: {log_topic}')

    def on_log(self, msg):
        now = time.time()
        for e in msg.event_log:
            n = self.by_name.get(e.node_name)
            if n is None:
                continue
            if n is self.root and e.current_status == 'RUNNING' and e.previous_status == 'IDLE':
                for m in walk(self.root):           # 새 도킹 시작 → 전부 초기화
                    m.status, m.t_start, m.t_end = 'IDLE', None, None
                self.dock_state = self.dock_retries = 0
            if e.current_status == 'RUNNING' and n.status != 'RUNNING':
                n.t_start, n.t_end = now, None
            if e.current_status in ('SUCCESS', 'FAILURE'):
                n.t_end = now
            if e.current_status != 'IDLE':        # 끝난 노드가 IDLE 로 돌아가도 결과 색은 남긴다
                n.status = e.current_status
                self.infer(n, now)

    def infer(self, n, now):
        """창을 도중에 켜서 앞선 로그를 놓쳤을 때를 메운다.
        자식이 움직였으면 부모는 실행 중이고, Sequence 에서 뒤 자식이 움직였으면 앞 자식은 성공한 것이다."""
        child, p = n, n.parent
        while p is not None:
            if p.status == 'IDLE':
                p.status, p.t_start = 'RUNNING', p.t_start or now
            if p.tag in ('Sequence', 'SequenceWithMemory', 'ReactiveSequence'):
                for sib in p.children[:p.children.index(child)]:
                    if sib.status in ('IDLE', 'RUNNING'):
                        for m in walk(sib):
                            if m.status in ('IDLE', 'RUNNING'):
                                m.status, m.t_end = 'SUCCESS', m.t_end or now
            child, p = p, p.parent

    def on_status(self, msg):
        if not msg.status_list:
            return
        st = msg.status_list[-1].status          # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        if st not in (4, 5, 6) or self.root.status != 'RUNNING':
            return
        result = 'SUCCESS' if st == 4 else 'FAILURE'
        now = time.time()
        for n in walk(self.root):                 # 아직 실행 중으로 남은 노드 = 마지막에 끝난 노드들
            if n.status == 'RUNNING':
                n.status, n.t_end = result, now

    def on_dock_fb(self, msg):
        self.dock_state = msg.feedback.state
        self.dock_retries = msg.feedback.num_retries

    def on_det(self, msg):
        self.last_det = time.time()
        self.det_z = msg.pose.position.z

    def draw(self):
        img = Image.new('RGB', (W, H), (250, 250, 250))
        d = ImageDraw.Draw(img)
        nodes = list(walk(self.root))
        for n in nodes:
            for c in n.children:
                d.line([(n.x, n.y + 32), (c.x, c.y - 32)], fill=(120, 120, 120), width=3)
        for n in nodes:
            control = n.tag in CONTROL_TAGS
            bw, bh = min(300, self.slot - 24), 64
            box = [n.x - bw / 2, n.y - bh / 2, n.x + bw / 2, n.y + bh / 2]
            outline = (40, 40, 40) if n.status == 'RUNNING' else (90, 90, 90)
            d.rounded_rectangle(box, radius=14 if control else 4, fill=COLORS[n.status],
                                outline=outline, width=5 if n.status == 'RUNNING' else 2)
            d.text((n.x, n.y - 10), n.name, font=self.f_mid, fill=(20, 20, 20), anchor='mm')
            sub = f'{n.tag}  ·  {STATUS_KO[n.status]}'
            if n.t_start:
                sub += f'  {((n.t_end or time.time()) - n.t_start):.1f} s'
            d.text((n.x, n.y + 17), sub, font=self.f_small, fill=(40, 40, 40), anchor='mm')
            if n.tag == 'DockRobot':
                info = f"dock_id: {n.attrs.get('dock_id', '')}"
                if n.status == 'RUNNING':
                    info += f'  |  {DOCK_STATE_KO.get(self.dock_state, "")}'
                    if self.dock_retries:
                        info += f'  재시도 {self.dock_retries}'
                d.text((n.x, n.y + 50), info, font=self.f_small, fill=(30, 30, 30), anchor='mm')
        running = [n for n in nodes if n.status == 'RUNNING' and not n.children]
        title = '도킹 BT — ' + (running[-1].name if running else STATUS_KO[self.root.status])
        d.text((20, 18), title, font=self.f_big, fill=(20, 20, 20))
        seen = time.time() - self.last_det < 0.25   # 카메라 30 Hz → 몇 장 연속으로 못 찾으면 "안 보임"
        mark = f'마커 보임  카메라-마커 {self.det_z:.2f} m' if seen else '마커 안 보임'
        d.rounded_rectangle([W - 380, 14, W - 20, 58], radius=8,
                            fill=(90, 190, 90) if seen else (200, 200, 200))
        d.text((W - 200, 36), mark, font=self.f_mid, fill=(20, 20, 20), anchor='mm')
        legend_x = 20
        for s in ('IDLE', 'RUNNING', 'SUCCESS', 'FAILURE'):
            d.rectangle([legend_x, H - 38, legend_x + 22, H - 16], fill=COLORS[s], outline=(90, 90, 90))
            d.text((legend_x + 30, H - 27), STATUS_KO[s], font=self.f_small, fill=(40, 40, 40), anchor='lm')
            legend_x += 110
        out = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        if self.scale != 1.0:
            out = cv2.resize(out, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        return out


def main():
    rclpy.init()
    node = BtViewer()
    cv2.namedWindow('Docking BT', cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            for _ in range(200):                 # 쌓인 메시지를 다 처리한다 (피드백 50 Hz + 검출 30 Hz)
                rclpy.spin_once(node, timeout_sec=0.0)
            cv2.imshow('Docking BT', node.draw())
            if cv2.waitKey(1) in (27, ord('q')) or cv2.getWindowProperty('Docking BT', 0) < 0:
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
