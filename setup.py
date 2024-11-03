import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'roboticpark_cyberattacks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raul',
    maintainer_email='r-a-c@users.noreply.github.com',
    description='Package dedicated to launch attacks against other systems',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dosattack = roboticpark_cyberattacks.dosattack:main',
            'logcapturer = roboticpark_cyberattacks.logcapturer:main',
            'replyattack = roboticpark_cyberattacks.replyattack:main',
            'replyattack_fake_node = roboticpark_cyberattacks.replyattack_fake_node:main'
        ],
    },
)
