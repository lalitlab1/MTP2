from setuptools import find_packages, setup

package_name = 'camera_utils'

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
    maintainer='friday',
    maintainer_email='lalitsaraf1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher_node = camera_utils.camera_publisher_node:main',
            'camera_sam_node = camera_utils.camera_sam_node:main',
        ],
    },
)
