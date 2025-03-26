from setuptools import setup
import os
from glob import glob

package_name = 'arc_detector_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fsae',
    maintainer_email='mah.yas.moussa@gmail.com',
    description='Detects partial circular arcs in LiDAR sheets.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arc_detector_node = arc_detector_node.arc_detector_node:main',
        ],
    },
)

