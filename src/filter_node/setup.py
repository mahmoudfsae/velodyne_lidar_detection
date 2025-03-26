from setuptools import find_packages, setup

package_name = 'filter_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fsae',
    maintainer_email='mah.yas.moussa@gmail.com',
    description='Node for filtering LiDAR data before arc detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_node = filter_node.filter_node:main',
        ],
    },
)

