from setuptools import setup

package_name = 'foxy_bridge_pkg'

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
            'foxy_scan_publisher = foxy_bridge_pkg.foxy_scan_publisher:main',
            'foxy_tf_publisher = foxy_bridge_pkg.foxy_tf_publisher:main'
        ],
    },
)

