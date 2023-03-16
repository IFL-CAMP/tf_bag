import setuptools


setuptools.setup(
    name='tf_bag',
    version='1.0.0',
    author='Marco Esposito',
    packages=setuptools.find_packages(where='src'),
    package_dir={'': 'src'}
)
