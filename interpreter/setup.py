from setuptools import find_packages, setup

package_name = 'interpreter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerome',
    maintainer_email='jer19801980@gmail.com',
    description='Parses atwork ObjectTask into 2D string array and republishes it',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'atwork_task_parser = interpreter.btt:main',
        ],
    },
)
