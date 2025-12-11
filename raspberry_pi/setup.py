from setuptools import setup

setup(
    name='signal_project',
    version='0.1.0',
    packages=['signal_project',
              'signal_project.audio_engine',
              'signal_project.state_machine',
              'signal_project.nodes'],
    package_dir={'': '.'},
    install_requires=[],
)

