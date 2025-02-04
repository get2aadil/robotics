from setuptools import find_packages, setup

package_name = 'image_capture_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python','cv_bridge'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_capture_node = image_capture_package.image_capture_node:main'
        ],
    },
)
