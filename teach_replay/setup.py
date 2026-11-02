from setuptools import find_packages, setup

package_name = 'teach_replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ICIC-Robot',
    maintainer_email='322911981+ICIC-Robot@users.noreply.github.com',
    description='Franka + XHand teach and replay (bag recording, B-spline + toppra, 1 kHz replay).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_demo = teach_replay.record_demo:main',
            'process_demo = teach_replay.process_demo:main',
            'replay_demo = teach_replay.replay_demo:main',
            'go_home = teach_replay.go_home:main',
        ],
    },
)
