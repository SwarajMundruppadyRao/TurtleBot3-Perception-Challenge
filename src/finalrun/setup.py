from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'finalrun'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),  # Include launch files
        (os.path.join('share', package_name), glob('scripts/*')),  # Include scripts if you have this directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swaraj',
    maintainer_email='143655010+SwarajMundruppadyRao@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enpm673_final_proj_main = finalrun.enpm673_final_proj_main:main',
            'horizon = finalrun.horizon:main',
            'optical_flow = finalrun.optical_flow:main',
            'stop_sign_detection = finalrun.stop_sign_detection:main',
            # Make sure all executable names are correctly assigned here
        ],
    },
)
