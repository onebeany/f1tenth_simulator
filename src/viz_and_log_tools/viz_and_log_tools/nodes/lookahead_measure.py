#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import datetime
import signal
import sys
import matplotlib.pyplot as plt
from collections import defaultdict
import csv
import os

class LookaheadMeasure(Node):
    def __init__(self):
        super().__init__('lookahead_measure')
        
        # 상수 정의
        self.START_X = 1.14523
        self.START_Y = 1.31397
        self.THRESHOLD = 0.5
        self.INITIAL_DELAY = 2.0
        
        # 데이터 저장 변수
        self.lap_data = defaultdict(lambda: {
            'time': [], 
            'velocity': [], 
            'curr_lookahead': [],
            'x': [], 
            'y': []
        })
        self.current_lap = 0
        self.start_time = None
        self.is_moving = False
        self.collecting_points = False
        
        # 구독 설정
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            50)
        
        self.lookahead_sub = self.create_subscription(
            Float64,
            '/curr_lookahead',
            self.lookahead_callback,
            50)
            
        # Drive 메시지 구독 추가
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            50)
            
        # Ctrl+C 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.current_velocity = 0.0
        self.current_lookahead = 0.0
        
    def drive_callback(self, msg):
        self.current_velocity = msg.drive.speed
        
    def odom_callback(self, msg):
        if not self.is_moving and abs(msg.twist.twist.linear.x) > 0:
            self.is_moving = True
            self.start_time = time.time()
            self.get_logger().info(f'Vehicle started moving at {self.start_time:.3f} seconds')
            return
            
        if not self.is_moving:
            return
            
        current_time = time.time() - self.start_time
        
        # 현재 위치 저장
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # 시작점과의 거리 계산
        distance = np.sqrt((current_x - self.START_X)**2 + (current_y - self.START_Y)**2)
        
        # 랩 체크 및 데이터 저장
        if distance <= self.THRESHOLD:
            if not self.collecting_points:
                self.collecting_points = True
                if self.current_lap > 0:
                    self.get_logger().info(f'Lap {self.current_lap} completed')
                self.current_lap += 1
        else:
            self.collecting_points = False
            
        self.lap_data[self.current_lap]['time'].append(current_time)
        self.lap_data[self.current_lap]['velocity'].append(self.current_velocity)
        self.lap_data[self.current_lap]['curr_lookahead'].append(self.current_lookahead)
        self.lap_data[self.current_lap]['x'].append(current_x)
        self.lap_data[self.current_lap]['y'].append(current_y)
    
    def lookahead_callback(self, msg):
        self.current_lookahead = msg.data
        
    def signal_handler(self, sig, frame):
        self.analyze_and_visualize_data()
        sys.exit(0)  
            
    def analyze_and_visualize_data(self):
        # 각 랩의 평균 curr_lookahead distance 계산
        lap_avg_lookaheads = {}
        for lap, data in self.lap_data.items():
            avg_lookahead = np.mean(data['curr_lookahead'])
            lap_avg_lookaheads[lap] = avg_lookahead
            self.get_logger().info(f"Lap {lap} - Average Lookahead Distance: {avg_lookahead:.3f}m")

        # 전체 랩의 평균 curr_lookahead distance 계산    
        total_avg_lookahead = np.mean(list(lap_avg_lookaheads.values()))
        self.get_logger().info(f"\nAverage Lookahead Distance across all laps: {total_avg_lookahead:.3f}m")

        # 각 랩의 curr_lookahead 데이터 표준편차 계산
        lap_lookahead_stds = {}
        for lap, data in self.lap_data.items():
            lookahead_std = np.std(data['curr_lookahead']) 
            lap_lookahead_stds[lap] = lookahead_std

        # 표준편차가 최대인 랩 선택    
        max_std_lap = max(lap_lookahead_stds, key=lap_lookahead_stds.get)
        self.get_logger().info(f"\nLap {max_std_lap} has the highest lookahead standard deviation of {lap_lookahead_stds[max_std_lap]:.3f}m")
        
        # 표준편차가 최대인 랩의 데이터 선택
        max_std_lap_data = self.lap_data[max_std_lap]
        
        # 그래프 그리기
        fig, ax1 = plt.subplots()
        
        # 첫 번째 y축 - curr_lookahead
        ax1.plot(max_std_lap_data['time'], max_std_lap_data['curr_lookahead'], color='b')  
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Lookahead Distance (m)', color='b')
        ax1.tick_params('y', colors='b')
        
        # 두 번째 y축 - velocity
        ax2 = ax1.twinx()
        ax2.plot(max_std_lap_data['time'], max_std_lap_data['velocity'], color='r')  
        ax2.set_ylabel('Velocity (m/s)', color='r')
        ax2.tick_params('y', colors='r')
        
        fig.tight_layout()
        plt.title(f"Lap {max_std_lap} - Lookahead Distance and Velocity")
        
        # 그래프 저장
        plt.savefig('lookahead_velocity_graph.png')
        self.get_logger().info("Lookahead and velocity graph saved as 'lookahead_velocity_graph.png'")
        
        # CSV 파일로 데이터 저장
        csv_filename = 'lookahead_data.csv'
        csv_headers = ['lap', 'time', 'velocity', 'lookahead', 'x', 'y']
        
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=csv_headers)
            writer.writeheader()
            
            for lap, lap_data in self.lap_data.items():
                for i in range(len(lap_data['time'])):
                    writer.writerow({
                        'lap': lap,
                        'time': lap_data['time'][i],
                        'velocity': lap_data['velocity'][i],
                        'lookahead': lap_data['curr_lookahead'][i],
                        'x': lap_data['x'][i],
                        'y': lap_data['y'][i]
                    })
                    
        self.get_logger().info(f"Lookahead data saved as '{csv_filename}'")
                
        # 파일 저장 경로 출력
        current_dir = os.path.abspath(os.path.dirname(__file__))
        self.get_logger().info(f"Files saved in: {current_dir}")
        

def main(args=None):
    rclpy.init(args=args)
    lookahead_measure = LookaheadMeasure()
    rclpy.spin(lookahead_measure)
    rclpy.shutdown()


if __name__ == '__main__':
    main()