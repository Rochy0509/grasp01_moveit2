from setuptools import find_packages, setup
import os

package_name = 'grasp01_robotics_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.py')]),
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', f) for f in os.listdir('config') if f.endswith('.yaml')]),
    ],
    install_requires=['setuptools', 'matplotlib','grasp01_moveit2','grasp01_description','grasp01_hardware'],
    zip_safe=True,
    maintainer='grasp',
    maintainer_email='kennethaldahir.martinezmoreno@ontariotechu.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp01_moveit_py = grasp01_robotics_experiments.grasp01_moveit_py:main',
            'grasp01_freq = grasp01_robotics_experiments.grasp01_frequency_response:main',

        ],
    },
)
