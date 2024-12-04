#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
import csv
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from os.path import expanduser
import argparse


class CSVMarkerPublisher(Node):
    def __init__(self, csv_file_path, topic_name, col):
        super().__init__('csv_marker_publisher')
        self.publisher = self.create_publisher(Marker, topic_name, 10)
        self.csv_file_path = csv_file_path
        self.col = col
        # 2초마다 read_csv_and_publish_markers 메서드를 호출하는 타이머 생성
        self.timer = self.create_timer(2.0, self.read_csv_and_publish_markers)

    def read_csv_and_publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # 포인트의 크기
        marker.scale.y = 0.2
        marker.color.a = 1.0  # 기본 알파 값

        with open(self.csv_file_path, 'r') as file:
            csv_reader = csv.reader(file, delimiter=',')
            header = next(csv_reader)  # 헤더 건너뛰기
            numCol = len(header)

            for row in csv_reader:
                x, y, speed = map(float, row)
                point = Point(x=x, y=y, z=0.0)
                color = ColorRGBA(a=1.0, r=0.0, g=self.col - speed/10, b=0.0)  # 속도에 따라 색상 조정
                marker.points.append(point)
                marker.colors.append(color)

        self.publisher.publish(marker)

def main(args=None):
    home = expanduser('~')
    parser = argparse.ArgumentParser(description='CSV Marker Publisher Node')
    parser.add_argument('--csv', type=str, help='Full path to the waypoints CSV file', required=True)
    parser.add_argument('--topic', type=str, default='visualization_marker', help='The topic name to publish Marker messages')
    parser.add_argument('--color', type=float, default=1.0, help='The color of Marker')
    # 커맨드라인에서 제공된 인자를 파싱
    arguments = parser.parse_args(args)
    csv_file_name = arguments.csv
    topic_name = arguments.topic
    col = arguments.color
    rclpy.init(args=args)
    csv_file_path = '/home/onebean/sim_ws/src/racelines/' + csv_file_name + '.csv' # 실제 CSV 파일 경로로 변경
    csv_marker_publisher = CSVMarkerPublisher(csv_file_path, topic_name, col)
    rclpy.spin(csv_marker_publisher)
    csv_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
