from setuptools import setup

setup(
    name='Sensor Data',
    version='1.0',
    description='Read data from GPS hat, heart rate monitor and power meter',
    author='jawn-smith',
    py_modules=['sensor_data'],
    entry_points = {
        'console_scripts': ['sensor_data=sensor_data:main'],
    }
)
