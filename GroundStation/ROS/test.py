import rclpy
import argparse
import sys
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import NavSatFix

class DronePublisher(Node):
    def __init__(self, drone_id, battery, lat, lon, alt, sat_count):
        super().__init__(f'drone_{drone_id}_publisher')
        
        # Publicadores
        self.battery_pub = self.create_publisher(Float64, f'/drone_{drone_id}/battery_level', 10)
        self.location_pub = self.create_publisher(NavSatFix, f'/drone_{drone_id}/location', 10)
        self.sat_pub = self.create_publisher(Int32, f'/drone_{drone_id}/satellite_count', 10)
        
        # Atributos
        self.drone_id = drone_id
        self.battery_val = battery
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.sat_count = sat_count
        
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f'Nó iniciado para Drone {self.drone_id}')

    def timer_callback(self):
        # Bateria
        bat_msg = Float64()
        bat_msg.data = float(self.battery_val)
        self.battery_pub.publish(bat_msg)
        
        # Localização
        nav_msg = NavSatFix()
        nav_msg.latitude = float(self.lat)
        nav_msg.longitude = float(self.lon)
        nav_msg.altitude = float(self.alt)
        self.location_pub.publish(nav_msg)
        
        # Satélites
        sat_msg = Int32()
        sat_msg.data = int(self.sat_count)
        self.sat_pub.publish(sat_msg)
        
        self.get_logger().info(f'[Drone {self.drone_id}] Enviando dados...')

def main(args=None):
    # Configuração do Argparse
    parser = argparse.ArgumentParser(description='Publicador de telemetria de Drone ROS2')
    parser.add_argument('--id', type=int, default=1, help='ID do drone')
    parser.add_argument('--bat', type=float, default=90.0, help='Nível da bateria')
    parser.add_argument('--lat', type=float, default=-22.9086254, help='Latitude')
    parser.add_argument('--lon', type=float, default=-43.1030033, help='Longitude')
    parser.add_argument('--alt', type=float, default=10.0, help='Altitude')
    parser.add_argument('--sats', type=int, default=12, help='Quantidade de satélites')
    
    # Filtramos os argumentos do ROS para o argparse não dar erro
    parsed_args, unknown = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=args)
    
    # Criamos o nó usando os argumentos processados
    node = DronePublisher(
        drone_id=parsed_args.id,
        battery=parsed_args.bat,
        lat=parsed_args.lat,
        lon=parsed_args.lon,
        alt=parsed_args.alt,
        sat_count=parsed_args.sats
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()