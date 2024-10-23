from setuptools import setup

package_name = 'map_manager_denm_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ettusb2102',
    maintainer_email='sourabha.lolage@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "denm_test_node = map_manager_denm_scripts.Denm_Publish_OSM:main",
            "cpm_test_node = map_manager_denm_scripts.Collision_Warning_CPM:main"
        ],
    },
)
