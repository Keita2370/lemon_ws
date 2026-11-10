#!/usr/bin/env python3

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


class LemonHarvestManager(Node):
    """
    各ノードのServiceを順に呼ぶだけの簡易シーケンサ (将来Behavior Treeに置換)。

        対象が固定されるのを待つ
        → /lemon_approach/approach  (固定位置をもとにLemon手前 (pregrasp) へ移動)
        → /lemon_grasp/grasp        (Servoで追従しながら掴み、pregraspへ戻る)
        → /arm_home/go_collect      (Lemon回収姿勢へ移動)
        → /lemon_grasp/open         (グリッパを開いてLemonを離す)
        → /arm_home/go_observe      (認識姿勢へ戻る。右/左は arm_home の今の向き。
                                     人が go_observe_right / go_observe_left で切り替える)
        → /lemon_target/release     (対象を解除。認識姿勢の視野で次の対象を選び直す)
        → 繰り返し

    approach / grasp に失敗した場合 (Lemonを持っていない) は回収姿勢へは行かず、
        → /arm_home/go_observe → /lemon_grasp/open → /lemon_target/release → 繰り返し

    起動時に一度 /arm_home/go_observe を呼んで認識姿勢へ行く (harvest_enabled が true に
    なり、対象の選択が始まる)。失敗したら waiting_target のまま待つので、
    手で /arm_home/go_observe を呼べば始まる。

    home (/arm_home/go_home) は犬が移動するときの畳んだ姿勢で、ここでは使わない。
    home の間は /arm_home/harvest_enabled が false で lemon_target が対象を固定しないので、
    waiting_target のまま待つ (go_home の後は自動では再開しない。手で go_observe を呼ぶ)。
    status は UI が既存の文字列で表示を分けているので変えない
    (回収姿勢・認識姿勢へ動く間も going_home)。
    """

    def __init__(self):
        super().__init__("lemon_harvest_manager")

        self.declare_parameter("call_timeout", 120.0)
        self.call_timeout = self.get_parameter("call_timeout").value

        self.cb_group = ReentrantCallbackGroup()

        self.target = None
        self.target_event = threading.Event()

        self.approach_client = self.create_client(
            Trigger, "/lemon_approach/approach", callback_group=self.cb_group
        )
        self.grasp_client = self.create_client(
            Trigger, "/lemon_grasp/grasp", callback_group=self.cb_group
        )
        self.collect_client = self.create_client(
            Trigger, "/arm_home/go_collect", callback_group=self.cb_group
        )
        self.open_client = self.create_client(
            Trigger, "/lemon_grasp/open", callback_group=self.cb_group
        )
        self.observe_client = self.create_client(
            Trigger, "/arm_home/go_observe", callback_group=self.cb_group
        )
        self.release_client = self.create_client(
            Trigger, "/lemon_target/release", callback_group=self.cb_group
        )

        self.status_pub = self.create_publisher(String, "~/status", 10)

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            PointStamped,
            "/lemon_target/target",
            self.target_callback,
            latched,
            callback_group=self.cb_group
        )

        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def target_callback(self, msg):

        # frame_idが空なら対象なし
        if not msg.header.frame_id:
            self.target = None
            self.target_event.clear()
            return

        self.target = msg
        self.target_event.set()

    # --------------------------------------------------

    def run(self):

        self.go_observe_on_startup()

        while rclpy.ok():

            self.publish_status("waiting_target")

            if not self.target_event.wait(timeout=1.0):
                continue

            p = self.target.point
            self.get_logger().info(
                f"Target ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )

            grasped = False

            self.publish_status("approaching")
            ok, message = self.call(self.approach_client)

            if ok:
                self.publish_status("grasping")
                ok, message = self.call(self.grasp_client)

                if ok:
                    self.get_logger().info("Grasped")
                    grasped = True
                else:
                    self.get_logger().warn(f"Grasp failed: {message}")

            else:
                self.get_logger().warn(f"Approach failed: {message}")

            if grasped:
                # 回収姿勢へ運んでLemonを離す
                self.publish_status("going_home")
                ok, message = self.call(self.collect_client)

                if not ok:
                    # Lemonを持ったまま動けないので、開かずに停止する
                    self.get_logger().error(
                        f"Go collect failed: {message}. Stopping sequence"
                    )
                    self.publish_status("error")
                    return

                self.publish_status("opening")
                ok, message = self.call(self.open_client)

                if not ok:
                    self.get_logger().warn(f"Open gripper failed: {message}")

            self.publish_status("going_home")
            ok, message = self.call(self.observe_client)

            if not ok:
                # 認識姿勢に戻れないと次の対象を正しく選べないので停止する
                self.get_logger().error(
                    f"Go observe failed: {message}. Stopping sequence"
                )
                self.publish_status("error")
                return

            if not grasped:
                # 把持に失敗していても閉じたままにしないよう開く
                self.publish_status("opening")
                ok, message = self.call(self.open_client)

                if not ok:
                    self.get_logger().warn(f"Open gripper failed: {message}")

            self.publish_status("releasing")
            self.target_event.clear()
            ok, message = self.call(self.release_client)

            if not ok:
                self.get_logger().warn(f"Release failed: {message}")

    def go_observe_on_startup(self):

        # arm_home が起動するまで待つ
        while rclpy.ok() and not self.observe_client.wait_for_service(timeout_sec=1.0):
            self.publish_status("waiting_target")

        self.publish_status("going_home")
        ok, message = self.call(self.observe_client)

        if ok:
            self.get_logger().info("Reached observe on startup")
        else:
            self.get_logger().error(
                f"Go observe on startup failed: {message}. "
                "Call /arm_home/go_observe to start"
            )

    def call(self, client):

        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"{client.srv_name} not available"

        future = client.call_async(Trigger.Request())

        event = threading.Event()
        future.add_done_callback(lambda _: event.set())

        if not event.wait(self.call_timeout):
            return False, "timeout"

        result = future.result()
        return result.success, result.message

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main():

    rclpy.init()

    node = LemonHarvestManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
