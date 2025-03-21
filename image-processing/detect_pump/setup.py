from setuptools import find_packages, setup

package_name = 'detect_pump'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duco',
    maintainer_email='vanuoc.phung@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = detect_pump.image_pub:main',
            'detect_pump = detect_pump.detect_pump:main',
        ],
    },
)
