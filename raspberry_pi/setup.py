from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'signal_project',
        'signal_project.led_engine',
        'signal_project.state_machine',
    ],
    package_dir={'': 'signal_project'}
)

setup(**d)
