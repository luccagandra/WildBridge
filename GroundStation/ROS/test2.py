#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Bool, Float64, String, Empty, Int32, Float64MultiArray
)
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Vector3


# ───────────────────────────────────────────────
# 📦 TEMPLATE DE TÓPICOS (drone_1 como base)
# ───────────────────────────────────────────────
TOPICS = {

    # Estado
    "/drone_1/altitude_reached": {"type": Bool, "default": True},
    "/drone_1/attitude": {"type": String, "default": "stable"},
    "/drone_1/battery_level": {"type": Float64, "default": 75.0},
    "/drone_1/battery_needed_to_go_home": {"type": Float64, "default": 30.0},
    "/drone_1/battery_needed_to_land": {"type": Float64, "default": 15.0},
    "/drone_1/camera/is_recording": {"type": Bool, "default": False},

    # Commands
    "/drone_1/command/abort_dji_native_mission": {"type": Empty, "default": None},
    "/drone_1/command/abort_mission": {"type": Empty, "default": None},
    "/drone_1/command/camera/start_recording": {"type": Empty, "default": None},
    "/drone_1/command/camera/stop_recording": {"type": Empty, "default": None},
    "/drone_1/command/enable_virtual_stick": {"type": Empty, "default": None},

    "/drone_1/command/gimbal_pitch": {"type": Float64, "default": -10.0},
    "/drone_1/command/gimbal_yaw": {"type": Float64, "default": 0.0},
    "/drone_1/command/goto_altitude": {"type": Float64, "default": 30.0},

    "/drone_1/command/goto_trajectory": {"type": String, "default": "circle"},
    "/drone_1/command/goto_trajectory_dji_native": {"type": String, "default": "waypoint_mission"},

    "/drone_1/command/goto_waypoint": {"type": Float64MultiArray, "default": [-22.9, -43.2, 30.0, 0.0]},
    "/drone_1/command/goto_waypoint_pid_tuning": {"type": Float64MultiArray, "default": [1.0, 0.0, 0.1]},

    "/drone_1/command/goto_yaw": {"type": Float64, "default": 90.0},
    "/drone_1/command/land": {"type": Empty, "default": None},
    "/drone_1/command/rth": {"type": Empty, "default": None},
    "/drone_1/command/set_rth_altitude": {"type": Float64, "default": 40.0},

    "/drone_1/command/stick": {"type": Float64MultiArray, "default": [0.0, 0.0, 0.0, 0.0]},
    "/drone_1/command/takeoff": {"type": Empty, "default": None},
    "/drone_1/command/zoom_ratio": {"type": Float64, "default": 2.0},

    # Telemetria
    "/drone_1/distance_to_home": {"type": Float64, "default": 120.0},
    "/drone_1/gimbal_attitude": {"type": String, "default": "level"},
    "/drone_1/gimbal_joint_attitude": {"type": String, "default": "0,0,0"},
    "/drone_1/gimbal_pitch": {"type": Float64, "default": -10.0},
    "/drone_1/gimbal_yaw": {"type": Float64, "default": 0.0},
    "/drone_1/heading": {"type": Float64, "default": 180.0},

    "/drone_1/home_location": {"type": NavSatFix, "default": {"lat": -22.9, "lon": -43.2, "alt": 10.0}},
    "/drone_1/home_set": {"type": Bool, "default": True},

    "/drone_1/hybrid_fl": {"type": Float64, "default": 35.0},
    "/drone_1/intermediary_waypoint_reached": {"type": Bool, "default": False},

    "/drone_1/location": {"type": NavSatFix, "default": {"lat": -22.9001, "lon": -43.2001, "alt": 25.0}},

    "/drone_1/max_radius_can_fly_and_go_home": {"type": Float64, "default": 500.0},
    "/drone_1/optical_fl": {"type": Float64, "default": 24.0},

    "/drone_1/remaining_flight_time": {"type": Float64, "default": 600.0},
    "/drone_1/satellite_count": {"type": Int32, "default": 14},

    "/drone_1/speed": {"type": Float64, "default": 5.0},
    "/drone_1/speed_vector": {"type": Vector3, "default": {"x": 1.0, "y": 0.0, "z": -0.2}},

    "/drone_1/time_needed_to_go_home": {"type": Float64, "default": 120.0},
    "/drone_1/time_needed_to_land": {"type": Float64, "default": 60.0},
    "/drone_1/time_to_landing_spot": {"type": Float64, "default": 90.0},

    "/drone_1/video_frames": {"type": Image, "default": None},

    "/drone_1/waypoint_reached": {"type": Bool, "default": False},
    "/drone_1/yaw_reached": {"type": Bool, "default": True},

    "/drone_1/zoom_fl": {"type": Float64, "default": 70.0},
    "/drone_1/zoom_ratio": {"type": Float64, "default": 2.0},

    # Mission (não depende de drone)
    "/mission/message_log": {"type": String, "default": "Mission started"},
    "/mission/relay_coordinates": {"type": NavSatFix, "default": {"lat": -22.9010, "lon": -43.2100, "alt": 30.0}},
    "/mission/relay_point_locked": {"type": Bool, "default": False},
}


# ───────────────────────────────────────────────
# 🔄 Substitui drone_1 → drone_N
# ───────────────────────────────────────────────
def apply_drone_namespace(topics_dict, drone_id):
    updated = {}

    for topic, config in topics_dict.items():
        if "/drone_1/" in topic:
            new_topic = topic.replace("/drone_1/", f"/drone_{drone_id}/")
        else:
            new_topic = topic

        updated[new_topic] = config

    return updated


# ───────────────────────────────────────────────
# 🚀 Nó
# ───────────────────────────────────────────────
class MockPublisher(Node):

    def __init__(self, topic, msg_type, value):
        super().__init__("mock_once_cli")

        self.publisher = self.create_publisher(msg_type, topic, 10)
        self.msg = self.build_message(msg_type, value)
        self.topic = topic

        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

    def build_message(self, msg_type, value):
        msg = msg_type()

        if isinstance(msg, Float64):
            msg.data = float(value)

        elif isinstance(msg, Bool):
            msg.data = bool(value)

        elif isinstance(msg, String):
            msg.data = str(value)

        elif isinstance(msg, Int32):
            msg.data = int(value)

        elif isinstance(msg, Empty):
            pass

        elif isinstance(msg, Float64MultiArray):
            msg.data = list(map(float, value))

        elif isinstance(msg, NavSatFix):
            msg.latitude = value["lat"]
            msg.longitude = value["lon"]
            msg.altitude = value["alt"]

        elif isinstance(msg, Vector3):
            msg.x = value["x"]
            msg.y = value["y"]
            msg.z = value["z"]

        elif isinstance(msg, Image):
            msg.height = 1
            msg.width = 1
            msg.encoding = "rgb8"
            msg.data = [0, 0, 0]

        return msg

    def publish_once(self):
        if self.published:
            return

        self.publisher.publish(self.msg)
        self.get_logger().info(f"Publicado UMA VEZ em {self.topic}")
        self.published = True
        rclpy.shutdown()


# ───────────────────────────────────────────────
# CLI
# ───────────────────────────────────────────────
def choose_topic():
    topics_list = list(TOPICS.keys())

    print("\nEscolha o tópico:\n")
    for i, t in enumerate(topics_list):
        print(f"[{i:02d}] {t}")

    idx = int(input("\nÍndice: "))
    topic = topics_list[idx]

    config = TOPICS[topic]
    msg_type = config["type"]
    default = config["default"]

    print(f"\nTipo: {msg_type.__name__}")
    print(f"Valor padrão: {default}")

    raw = input("\nValor (Enter = padrão): ")

    if raw.strip() == "":
        return topic, msg_type, default

    return topic, msg_type, parse_input(msg_type, raw)


def parse_input(msg_type, raw):
    if msg_type == Float64:
        return float(raw)

    if msg_type == Bool:
        return raw.lower() in ["true", "1", "yes"]

    if msg_type == String:
        return raw

    if msg_type == Int32:
        return int(raw)

    if msg_type == Float64MultiArray:
        return [float(x) for x in raw.split(",")]

    if msg_type == NavSatFix:
        lat, lon, alt = map(float, raw.split(","))
        return {"lat": lat, "lon": lon, "alt": alt}

    if msg_type == Vector3:
        x, y, z = map(float, raw.split(","))
        return {"x": x, "y": y, "z": z}

    return None


# ───────────────────────────────────────────────
# MAIN
# ───────────────────────────────────────────────
def main():
    rclpy.init()

    drone_id = input("Digite o ID do drone (N): ").strip()

    if not drone_id.isdigit():
        print("Erro: N deve ser numérico")
        return

    global TOPICS
    TOPICS = apply_drone_namespace(TOPICS, drone_id)

    topic, msg_type, value = choose_topic()

    print("\nResumo:")
    print(topic, msg_type.__name__, value)

    input("\nENTER para publicar...")

    node = MockPublisher(topic, msg_type, value)
    rclpy.spin(node)


if __name__ == "__main__":
    main()