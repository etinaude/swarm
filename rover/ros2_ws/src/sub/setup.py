from setuptools import find_packages, setup

package_name = 'sub'

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
    maintainer='etienne',
    maintainer_email='eti@naude.dev',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
                'image_sub = sub.image_sub:main',
                'keyboard_sub = sub.keyboard_sub:main',
        ],
    },
)
