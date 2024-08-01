from setuptools import setup

package_name = 'sonic_py_movement'

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
    maintainer='aqua',
    maintainer_email='aqua@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'wanderer = sonic_py_movement.wanderer:main',
                'wanderer_listen = sonic_py_movement.wanderer_listen:main',
                'wanderer_listen_2 = sonic_py_movement.wanderer_listen_2:main',
                'reactor = sonic_py_movement.reactor:main',
                'listener = sonic_py_movement.subscriber_member_function:main',
                'talker = sonic_py_movement.publisher_member_function:main',
        ],
    },
)
