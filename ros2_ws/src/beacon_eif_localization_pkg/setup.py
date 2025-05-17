import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'beacon_eif_localization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angela Rider Jimenez / Jose Francisco Lopez Ruiz',
    maintainer_email='josloprui6@alum.us.es',
    description='Localizacion mediante balizas basada en el filtro extendido de informacion',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'drone_control_node = beacon_eif_localization_pkg.drone_control_node:main',
            'tf_manager_node = beacon_eif_localization_pkg.tf_manager_node:main',
            'visualization_node = beacon_eif_localization_pkg.visualization_node:main',
            'EIF_filter_node = beacon_eif_localization_pkg.EIF_filter_node:main',
        ],
    },
)
