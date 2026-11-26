from setuptools import setup

package = 'piper_x_control'
setup(
    name=package,
    version='0.1.0',
    packages=["piper_x_driver"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/config', ['config/driver.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={'console_scripts': ['motion_service = piper_x_driver.ros_motion_service:main']},
)
