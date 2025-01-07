from setuptools import setup

package_name = 'yolo_multi_cam'

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
    maintainer='Lorenz Gunreben',
    maintainer_email='gunreben@wifa.uni-leipzig.de',
    description='YOLO Multi-Cam-Packages for people detection around agricultural machinery',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'multi_cam_yolo = yolo_multi_cam.multi_cam_yolo:main'
        ],
    },
)

