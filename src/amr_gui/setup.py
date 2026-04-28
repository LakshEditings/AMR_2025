from setuptools import setup

package_name = 'amr_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@amr.com',
    description='Custom tkinter GUI for AMR remote control with LiDAR visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amr_gui_node = amr_gui.amr_gui_node:main',
        ],
    },
)
