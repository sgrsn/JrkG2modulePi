from setuptools import setup, find_packages

__auther__ = 'Hidaka Sato'

setup(
    name = 'JrkG2modulePi',
    version = '1.1.0',
    description = 'The Jrk G2 library for Linux, RaspberryPi.',
    auther = 'Hidaka Sato',
    url = 'https://github.com/sgrsn/JrkG2modulePi',
    classifiers = [
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 2.7'
        ],
    packages = find_packages(),
    include_package_data = True,
    keywords = ['Jrk', 'Jrk G2', 'Pololu'],
    license = 'MIT License',
    install_requires = [
        'pyserial',
        'smbus2',
        'enum34'
        ]
)
