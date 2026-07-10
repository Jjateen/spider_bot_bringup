import os
from setuptools import find_packages, setup

package_name = 'ahrs'


def _collect_files(src_dir, dest_dir):
    result = []
    for root, dirs, files in os.walk(src_dir):
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        target = os.path.join(dest_dir, rel) if rel != '.' else dest_dir
        paths = [os.path.join(root, f) for f in files]
        result.append((target, paths))
    return result


config_files = _collect_files('config', os.path.join('share', package_name, 'config'))
launch_files = _collect_files('launch', os.path.join('share', package_name, 'launch'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + config_files + launch_files,
    scripts=[
        'scripts/publish_test_imu.py',
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'pyyaml'],
    zip_safe=True,
    maintainer='jayesh',
    maintainer_email='scientistn1420@gmail.com',
    description='Real-time AHRS 3D visualizer using Open3D for ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ahrs_visualizer = ahrs.main:main',
        ],
    },
)
