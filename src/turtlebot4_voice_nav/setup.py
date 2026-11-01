from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

name = 'turtlebot4_voice_nav'
validation_data = [
    ('share/' + name + '/' + str(directory),
     [str(path) for path in directory.iterdir() if path.is_file()])
    for directory in [Path('validation'), *sorted(
        path for path in Path('validation').rglob('*') if path.is_dir())]
]
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'VALIDATION.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/config/generated', glob('config/generated/*')),
    ] + validation_data,
    scripts=['scripts/setup_voice.sh'], install_requires=['setuptools'],
    extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='lenovo', maintainer_email='hongquang100805@gmail.com',
    license='Apache-2.0', description='Semantic map regions and Vietnamese voice navigation',
    entry_points={'console_scripts': [
        'segment_map = turtlebot4_voice_nav.map_regions:main',
        'voice_navigator = turtlebot4_voice_nav.voice_navigator:main',
        'semantic_visualizer = turtlebot4_voice_nav.semantic_visualizer:main',
        'speech_recognition_node = turtlebot4_voice_nav.speech_recognition_node:main',
    ]})
