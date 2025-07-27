from setuptools import find_packages, setup

package_name = 'test_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intelligent Systems Lab',
    maintainer_email='isl.torvergata@gmail.com',
    description='Action and service tester server.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_server = test_server.test_server:main'
        ],
    },
)
