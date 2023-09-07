from setuptools import setup

package_name = 'my_package'

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
            'my_node = my_package.my_node:main',
            'minimal_service = my_package.minimal_service:main',
            'minimal_client = my_package.minimal_client:main',
            'minimal_publisher = my_package.minimal_publisher:main',
            'minimal_subscriber = my_package.minimal_subscriber:main'     
        ],
    },
)
