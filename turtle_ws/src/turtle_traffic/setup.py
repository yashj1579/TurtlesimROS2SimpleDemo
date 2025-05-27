from setuptools import setup

package_name = 'turtle_traffic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yash Jain',
    maintainer_email='your_email@example.com',
    description='A package for controlling turtle movement and traffic logic in a game of tag',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perform_movement = turtle_traffic.perform_movement:main',
            'point_gen = turtle_traffic.point_gen:main',
            'turtle_move = turtle_traffic.turtle_move:main',
        ],
    },
)
