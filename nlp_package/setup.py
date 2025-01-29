from setuptools import find_packages, setup

package_name = 'nlp_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #data_files=[
    #    ('share/ament_index/resource_index/packages',
    #        ['resource/' + package_name]),
    #    ('share/' + package_name, ['package.xml']),
    #],
    install_requires=['setuptools','openai','python-dotenv'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='NLP Node using Open AI API',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nlp_node = nlp_package.nlp_node:main'
        ],
    },
)
