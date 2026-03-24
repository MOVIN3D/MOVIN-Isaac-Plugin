from setuptools import setup, find_packages

setup(
    name='movin-isaac-plugin',
    version='0.1.0',
    packages=find_packages(where='movin_sdk_python', include=['movin_sdk_python', 'movin_sdk_python.*']),
    package_dir={'': 'movin_sdk_python'},
    package_data={
        'movin_sdk_python': ['**/*.xml', '**/*.json', '**/*.STL'],
    },
)
