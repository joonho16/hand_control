from setuptools import setup
import os
from glob import glob

package_name = 'hand_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- [1] Launch 파일 설치 설정 ---
        # launch 폴더 안의 모든 .launch.py 파일을 share/패키지명/launch 로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- [2] UI 파일 설치 설정 ---
        # ui 폴더 안의 모든 .ui 파일을 share/패키지명/ui 로 복사
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='STM32 Hand Control Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '실행명령어 = 패키지폴더.파이썬파일:main함수'
            'dxl_driver = hand_control.dxl_node:main',
            'hand_gui = hand_control.gui_node:main',
        ],
    },
)