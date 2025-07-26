from setuptools import setup

package_name = 'ros2_sender_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Send /chatter messages via UDP socket',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_sender = ros2_sender_pkg.scan_sender:main',
            'tf_sender = ros2_sender_pkg.tf_sender:main'
        ],
    },
)

