
import setuptools
import os
import glob


package_name = 'lop2'


setuptools.setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] ),
        ('share/' + package_name, ['package.xml'] ),

        # Launch files:
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py') ),

        # Data files:
        (os.path.join('share', package_name, 'data'),
           glob.glob('data/*') ),

        # RVIZ config files:
        (os.path.join('share', package_name, 'config'),
           glob.glob('config/*') ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lop_node = lop2.lop_node:main',
            'live_feed = lop2.live_feed:main',
        ],
    },
)
