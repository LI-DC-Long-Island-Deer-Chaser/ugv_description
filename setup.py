from setuptools import setup
import os
from glob import glob

package_name = 'ugv_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'slam'), glob('launch/slam/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config', 'sim'), glob('config/sim/*.yaml') + glob('config/sim/*.rviz') + glob('config/sim/*.parm')),
        (os.path.join('share', package_name, 'config', 'sim', 'cubeFCParams'), glob('config/sim/cubeFCParams/*.param')),
        (os.path.join('share', package_name, 'config', 'cartographer'), glob('config/cartographer/*.lua') + glob('config/cartographer/*.rviz')),
        (os.path.join('share', package_name, 'config', 'irl'), glob('config/irl/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
