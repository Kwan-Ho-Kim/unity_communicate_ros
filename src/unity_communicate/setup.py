from setuptools import find_packages, setup

package_name = 'unity_communicate'

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
    maintainer='root',
    maintainer_email='aarony12@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_sub = unity_communicate.image_sub:main',
            'footprint_maker = unity_communicate.footprint_maker:main',
            'path_reader = unity_communicate.path_reader:main',
            'path_recoder = unity_communicate.path_recoder:main'
        ],
    },
)
