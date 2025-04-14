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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angela Rider Jimenez / Jose Francisco Lopez Ruiz',
    maintainer_email='josloprui6@alum.us.es',
    description='Localizacion mediante balizas basada en el filtro extendido de informacion',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
