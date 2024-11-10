from setuptools import setup

package_name = 'reinforcement_learning_drive_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdw',
    maintainer_email='msd030428@gmail.com',
    description='A ROS2 package to subscribe to initial_pose topic and store data in an array',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_subscriber = reinforcement_learning_drive_model.initial_pose_subscriber:main',
            'path_publisher = reinforcement_learning_drive_model.path_publisher:main',
            'dqn_node = reinforcement_learning_drive_model.dqn_node:main',
            'dqn_single_node = reinforcement_learning_drive_model.dqn_single_node:main',
        ],
    },
)
