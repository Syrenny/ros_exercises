from setuptools import find_packages, setup

package_name = 'message_turtle_commands'

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
    maintainer='ilyusha',
    maintainer_email='Syrenny@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'action_turtle_server = message_turtle_commands.action_server:main',
        'action_turtle_client = message_turtle_commands.action_client:main',
        ],
    },
)
