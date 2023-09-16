from setuptools import setup

package_name = 'vision_system_pkg'

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
    maintainer='rock',
    maintainer_email='feltona6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streamer_node = vision_system_pkg.streamer_node:main',
            'hmi_node = vision_system_pkg.hmi_node:main'
        ],
    },
)
