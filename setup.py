from setuptools import setup

package_name = 'my_ros2_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lej',
    maintainer_email='lej@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'watch_tv = my_ros2_project.watch_tv:main',
            'way = my_ros2_project.way:main',
            'robot_start_speak = my_ros2_project.robot_start_speak:main',
            'robot_end_speak = my_ros2_project.robot_end_speak:main',
            'move = my_ros2_project.move:main',
            'rasp_cmd = my_ros2_project.rasp_cmd:main',
        ],
    },
)
