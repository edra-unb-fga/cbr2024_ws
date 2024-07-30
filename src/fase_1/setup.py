from setuptools import setup

package_name = 'fase_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu.email@exemplo.com',
    description='Descrição do seu pacote fase_1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fase1_script = fase_1.fase1_script:main',
        ],
    },
)
