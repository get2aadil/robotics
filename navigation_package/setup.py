from setuptools import find_packages, setup

package_name = 'navigation_package'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Navigation Node',
    license='Apache 2.0 License',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = navigation_package.navigation_node:main'
        ],
    },
)
