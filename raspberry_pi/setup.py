from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'signal_project',
        'signal_project.state_machine',
        'signal_project.state_machine.states',
        'signal_project.nodes',
        'signal_project.audio_engine',
        'signal_project.led_engine',
        'signal_project.scripts'
    ],
    package_dir={'': '.'}
)

setup(**d)
