# 패키지 빌드 도구 가져오기
from setuptools import setup, find_packages

# 패키지 이름 정의
package_name = 'viz_and_log_tools'

setup(
    # 패키지 기본 정보
    name=package_name,        # 패키지 이름
    version='0.0.1',         # 패키지 버전
    packages=find_packages(), # 자동으로 모든 Python 패키지 찾기
    
    # 패키지와 함께 설치될 데이터 파일
    data_files=[
        # ROS2 패키지 인덱스에 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 파일 위치 지정
        ('share/' + package_name, ['package.xml']),
    ],
    
    # 의존성 정의
    install_requires=['setuptools'],  # 필수 Python 패키지
    zip_safe=True,                   # ZIP 아카이브로 설치 가능
    
    # 패키지 메타데이터
    maintainer='onebean',
    maintainer_email='your.email@example.com',
    description='Visualization and logging tools for F1TENTH',
    license='TODO: License declaration',
    tests_require=['pytest'],        # 테스트 의존성
    
    # 실행 가능한 노드 정의
    entry_points={
        'console_scripts': [
            # 노드이름 = 패키지.모듈경로:시작함수
            'lap_time_measure = viz_and_log_tools.nodes.lap_time_measure:main',
        ],
    },
)
