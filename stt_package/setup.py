from setuptools import setup
import os
from glob import glob

package_name = 'stt_package'

def get_model_files():
    model_dir = 'stt_package/models/vosk-model-small-en-us-0.15'
    paths = []
    for (path, directories, filenames) in os.walk(model_dir):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            relative_path = os.path.relpath(full_path,model_dir)
            destination = os.path.join('share', package_name, 'models', 'vosk-model-small-en-us-0.15', os.path.dirname(relative_path))
            paths.append((destination, [full_path]))
    
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        *get_model_files(),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vosk', 'pyaudio', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Speech-to-Text Node using VOSK',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'stt_node = stt_package.stt_node:main'
        ],
    },
)
