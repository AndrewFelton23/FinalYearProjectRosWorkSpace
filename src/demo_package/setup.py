from setuptools import setup

package_name = 'demo_package'

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
    maintainer_email='rock@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = demo_package.my_node:main',
            'streamer_node = demo_package.streamer_node:main',
            'flask_node = demo_package.flask_node:main',
            'hmi_node = demo_package.hmi_node:main',
            'stack_node = demo_package.stack_node:main'
        ],
    },
)
