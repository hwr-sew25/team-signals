from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'signal_project',
        'signal_project.scripts',
        'signal_project.state_machine',
        'signal_project.nodes',
        'signal_project.audio_engine
    ],
    package_dir={'': 'signal_project'}
)

setup(**d)
