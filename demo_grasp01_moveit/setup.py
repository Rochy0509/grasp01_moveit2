from setuptools import find_packages, setup

package_name = 'demo_grasp01_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grasp01_subscriber.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grasp',
    maintainer_email='kennethaldahir.martinezmoreno@ontariotechu.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp01_subscriber = demo_grasp01_moveit.grasp01_planning_subscriber:main',
            'grasp01_rmd = demo_grasp01_moveit.grasp01_motors:main',

        ],
    },
)
