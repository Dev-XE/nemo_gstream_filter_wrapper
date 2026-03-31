from setuptools import find_packages, setup

package_name = 'nemo_gstream_filter_wrapper'

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
    maintainer='yashas',
    maintainer_email='yashas.manjunath007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'receiver = nemo_gstream_filter_wrapper.receiver:main',
            'receiver1 = nemo_gstream_filter_wrapper.receiver:main',
        ],
    },
)
