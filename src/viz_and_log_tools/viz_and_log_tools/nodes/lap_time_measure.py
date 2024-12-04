#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
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

save_dir = f'/home/onebean/sim_ws/laptime_result/{time.strftime("%Y%m%d_%H%M%S")}'
os.makedirs(save_dir, exist_ok=True)

class LapTimeMeasure(Node):
    def __init__(self):
        super().__init__('lap_time_measure')
        
        self.START_X = 1.03035
        self.START_Y = 0.982781
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
        
        # 변수 초기화
        self.lap_times = []
        self.start_time = None
        self.is_moving = False
        self.closest_point = None
        self.collecting_points = False
        
        # 오도메트리 구독 (25Hz)
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            25)
        
        # Lookahead 구독
        self.lookahead_sub = self.create_subscription(
            Float32,
            '/curr_lookahead',
            self.lookahead_callback,
            25)
            
        # Drive 메시지 구독
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            25)
        
        # Ctrl+C 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.current_velocity = 0.0
        self.current_lookahead = 0.0
        
        self.get_logger().info("LapTimeMeasure node initialized")
    
    def drive_callback(self, msg):
        self.current_velocity = msg.drive.speed
    
    def lookahead_callback(self, msg):
        self.current_lookahead = msg.data
        self.get_logger().debug(f'Received lookahead: {self.current_lookahead}')
    
    def odom_callback(self, msg):
        current_time = time.time()
        
        # 차량이 움직이기 시작했는지 확인
        if not self.is_moving and abs(msg.twist.twist.linear.x) > 0:
            self.is_moving = True
            self.start_time = current_time
            self.get_logger().info('Vehicle started moving at {:.3f} seconds'.format(current_time))
            return
        if not self.is_moving:
            return
        
        # 초기 지연 시간 이후에만 랩타임 측정
        if len(self.lap_times) == 0 and (current_time - self.start_time) <= self.INITIAL_DELAY:
            return
            
        # 현재 위치와 시작점 사이의 거리 계산
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        distance = np.sqrt((current_x - self.START_X)**2 + (current_y - self.START_Y)**2)

        # 현재 랩 데이터 저장
        current_lap = len(self.lap_times)
        self.lap_data[current_lap]['time'].append(current_time-self.start_time)
        self.lap_data[current_lap]['velocity'].append(self.current_velocity)
        self.lap_data[current_lap]['curr_lookahead'].append(float(self.current_lookahead))
        self.lap_data[current_lap]['x'].append(current_x)
        self.lap_data[current_lap]['y'].append(current_y)
        
        # 시작점 근처에서 포인트 수집
        if distance <= self.THRESHOLD:
            self.collecting_points = True
            if self.closest_point is None or distance < self.closest_point['distance']:
                self.closest_point = {
                    'time': current_time,
                    'distance': distance,
                    'position': (current_x, current_y)
                }
        else:
            # 시작점에서 벗어났을 때 가장 가까운 포인트 처리
            if self.collecting_points and self.closest_point is not None:
                if self.start_time is None:
                    lap_time = self.closest_point['time']
                else:
                    lap_time = self.closest_point['time'] - self.start_time
                    
                self.lap_times.append(lap_time)
                self.start_time = self.closest_point['time']
                
                self.get_logger().info(f'Lap {len(self.lap_times)} completed: {lap_time:.3f} seconds')
                self.get_logger().info(f'Closest point: x={self.closest_point["position"][0]:.3f}, y={self.closest_point["position"][1]:.3f}')
                
                # 상태 초기화
                self.collecting_points = False
                self.closest_point = None

    def signal_handler(self, sig, frame):
        if len(self.lap_times) > 0:
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(save_dir, f'laptime_{current_time}.csv')
            with open(filename, 'w') as f:
                f.write('lap_count,lap_time\n')
                for i, lap_time in enumerate(self.lap_times, 1):
                    f.write(f'{i},{lap_time:.3f}\n')
            self.get_logger().info(f'Lap times saved to {filename}')
            self.analyze_and_visualize_data()
        sys.exit(0)
        
    def analyze_and_visualize_data(self):
        
        if self.lap_data:
            last_lap = max(self.lap_data.keys())
            self.lap_data.pop(last_lap)  # 마지막 랩 데이터 제거
        
        # 디버깅 정보 출력
        for lap, data in self.lap_data.items():
            self.get_logger().info(f"Lap {lap+1} data sizes:")
            self.get_logger().info(f"  Time: {len(data['time'])}")
            self.get_logger().info(f"  Velocity: {len(data['velocity'])}")
            self.get_logger().info(f"  Lookahead: {len(data['curr_lookahead'])}")
            if len(data['curr_lookahead']) > 0:
                self.get_logger().info(f"  Sample lookahead values: {data['curr_lookahead'][:5]}")
        
        # 각 랩의 평균 curr_lookahead distance 계산
        lap_avg_lookaheads = {}
        for lap, data in self.lap_data.items():
            if len(data['curr_lookahead']) > 0:  # 데이터가 있는 경우에만 계산
                avg_lookahead = np.mean(data['curr_lookahead'])
                lap_avg_lookaheads[lap] = avg_lookahead
                self.get_logger().info(f"Lap {lap+1} - Average Lookahead Distance: {avg_lookahead:.3f}m")

        if lap_avg_lookaheads:  # 데이터가 있는 경우에만 계산
            # 전체 랩의 평균 curr_lookahead distance 계산    
            total_avg_lookahead = np.mean(list(lap_avg_lookaheads.values()))
            self.get_logger().info(f"\nAverage Lookahead Distance across all laps: {total_avg_lookahead:.3f}m")

            # 랩타임이 가장 빠른 랩 선택
            fastest_lap = np.argmin(self.lap_times)  # 마지막 랩은 제외
            self.get_logger().info(f"\nLap {fastest_lap+1} is the fastest lap: {self.lap_times[fastest_lap]:.3f}s")
            
            # 그래프 그리기
            fastest_lap_data = self.lap_data[fastest_lap]
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
            
            # lookahead distance 그래프
            ax1.plot(fastest_lap_data['time'], fastest_lap_data['curr_lookahead'], 'b-', 
                    label='Lookahead Distance')
            ax1.set_ylabel('Lookahead Distance (m)')
            ax1.grid(True)
            ax1.legend()
            
            # velocity 그래프
            ax2.plot(fastest_lap_data['time'], fastest_lap_data['velocity'], 'r-', 
                    label='Velocity')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.grid(True)
            ax2.legend()
            
            plt.suptitle(f"Lap {fastest_lap+1} (Fastest) - Lookahead Distance and Velocity")
            plt.tight_layout()
            
            # 그래프 저장
            graph_filename = os.path.join(save_dir, 'lookahead_velocity_graph.png')
            plt.savefig(graph_filename)
            self.get_logger().info(f"Lookahead and velocity graph saved as '{graph_filename}'")
            
            # CSV 파일로 데이터 저장
            csv_filename = os.path.join(save_dir, 'lookahead_data.csv')
            csv_headers = ['lap', 'time', 'velocity', 'lookahead', 'x', 'y']
            
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=csv_headers)
                writer.writeheader()
                
                for lap, lap_data in self.lap_data.items():
                    for i in range(len(lap_data['time'])):
                        writer.writerow({
                            'lap': lap+1,
                            'time': lap_data['time'][i],
                            'velocity': lap_data['velocity'][i],
                            'lookahead': lap_data['curr_lookahead'][i],
                            'x': lap_data['x'][i],
                            'y': lap_data['y'][i]
                        })
                        
            self.get_logger().info(f"Lookahead data saved as '{csv_filename}'")
            
def main(args=None):
    rclpy.init(args=args)
    lap_time_measure = LapTimeMeasure()
    rclpy.spin(lap_time_measure)
    rclpy.shutdown()

if __name__ == '__main__':
    main()