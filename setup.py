from setuptools import setup, find_packages

__auther__ = 'Hidaka Sato'

setup(
    name = 'JrkG2modulePi',
    version = '1.0.5',
    description = 'The JrkG2 library for RaspberryPi.',
    auther = 'Hidaka Sato',
    auther_email = 'sato@suzakugiken.jp',
    url = 'https://github.com/sgrsn/JrkG2modulePi',
    classifiers = [
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 2.7'
        ],
    packages = find_packages(),
    include_package_data = True,
    keywords = ['Jrk', 'JrkG2', 'Pololu', 'Suzakulab'],
    license = 'MIT License',
    install_requires = [
        'pyserial'
        'smbus2',
        'enum34'
        ]
)