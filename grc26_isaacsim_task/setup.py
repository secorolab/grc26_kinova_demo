from glob import glob
import os
from os.path import join as os_join
from setuptools import find_packages, setup

package_name = 'grc26_isaacsim_task'

def get_data_files(source_dir, install_dir):
    data_files = []
    for (path, dirs, filenames) in os.walk(source_dir):
        if not filenames:
            continue
        files = [os_join(path, f) for f in filenames]
        # Preserve subdirectory structure in install path
        rel_path = os.path.relpath(path, source_dir)
        if rel_path == '.':
            dest = install_dir
        else:
            dest = os_join(install_dir, rel_path)
        data_files.append((dest, files))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os_join('launch', '*'))),
        ('share/' + package_name + '/models', glob(os_join('models', '*'))),
        ('share/' + package_name + '/scripts', glob(os_join('scripts', '*'))),
        ('share/' + package_name + '/config', glob(os_join('config', '*'))),
        ('share/' + package_name + '/urdf', glob(os_join('urdf', '*'))),
        *get_data_files('assets', 'share/' + package_name + '/assets'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='batsy',
    maintainer_email='vamsikalagaturu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dualarm_pick_place = grc26_isaacsim_task.dualarm_pick_place:main',
        ],
    },
)
