from setuptools import find_packages, setup

package_name = 'ctrl_vel_motores'

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
    maintainer='Javier',
    maintainer_email='"javodiaz101@gmail.com"',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocidad_teclado = ctrl_vel_motores.velocidad_teclado:main',
            'velocidad_motores = ctrl_vel_motores.velocidad_motores:main',
        ],
    },
)
