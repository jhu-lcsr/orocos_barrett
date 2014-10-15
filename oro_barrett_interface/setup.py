from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/bhand_action_server.py'],
    packages=['oro_barrett_interface'],
    package_dir={'': 'src'}
)

