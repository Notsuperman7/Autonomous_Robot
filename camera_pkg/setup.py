from glob import glob

from setuptools import find_packages, setup
import os

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models/best_ncnn_model'), [
            'models/best_ncnn_model/metadata.yaml',
            'models/best_ncnn_model/model.ncnn.bin',
            'models/best_ncnn_model/model.ncnn.param',
            'models/best_ncnn_model/model_ncnn.py',
        ]),
        (os.path.join('share', package_name, 'models/best_yolov8n_ncnn_model'), [
            'models/best_yolov8n_ncnn_model/metadata.yaml',
            'models/best_yolov8n_ncnn_model/model.ncnn.bin',
            'models/best_yolov8n_ncnn_model/model.ncnn.param',
            'models/best_yolov8n_ncnn_model/model_ncnn.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nour',
    maintainer_email='nour@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = camera_pkg.detector_node:main',
            'camera_node = camera_pkg.image_publisher:main',
            'csi_camera_node = camera_pkg.csi_camera_node:main',
            'lane_detection_node = camera_pkg.lane_detection:main',
            "test_lane_detection = camera_pkg.test:main",
        ],
    },
)
