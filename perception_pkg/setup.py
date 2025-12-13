from setuptools import setup, find_packages
import os

package_name = 'sensor_fusion_cpm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',   # for cv_bridge usage (safe fallback)
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Perception fusion node with depth projection and RViz MarkerArray visualization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_fusion_node = sensor_fusion_cpm.perception_fusion_node:main',
        ],
    },
)