from setuptools import setup

package_name = 'rl_quadrotor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    description='RL quadrotor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_node = rl_quadrotor.game:main',
        ],
    },
)
