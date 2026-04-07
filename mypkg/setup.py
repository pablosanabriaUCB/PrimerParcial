from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pablo.sanabria@ucb.edu.bo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nodo1 = mypkg.nodo1:main',
            'nodo2 = mypkg.nodo2:main',
            'nodo3 = mypkg.nodo3:main',
            'nodo4 = mypkg.nodo4:main',
            'nodo5 = mypkg.nodo5:main',
        ],
    },
)
