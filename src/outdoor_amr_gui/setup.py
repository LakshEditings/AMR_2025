from setuptools import setup

package_name = 'outdoor_amr_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', ['resource/web/index.html']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@amr.com',
    description='Outdoor AMR GUI with GPS, 3D LiDAR and custom 3D Web View',
    license='MIT',
    entry_points={
        'console_scripts': [
            'outdoor_web_server = outdoor_amr_gui.outdoor_web_server:main',
            'outdoor_gui_node   = outdoor_amr_gui.outdoor_gui_node:main',
        ],
    },
)
