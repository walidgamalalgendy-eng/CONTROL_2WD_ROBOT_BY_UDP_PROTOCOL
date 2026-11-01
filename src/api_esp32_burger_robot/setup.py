from setuptools import find_packages, setup

package_name = 'api_esp32_burger_robot'

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
    maintainer='walied',
    maintainer_email='walied@todo.todo',
    description='API ESP32 Burger Robot Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_1_pub = api_esp32_burger_robot.task_1_pub:main',
            'task2_teleport = api_esp32_burger_robot.task2_teleport:main',
            'Task3_bot=api_esp32_burger_robot.Task3_bot:main',
        ],
    },
)
