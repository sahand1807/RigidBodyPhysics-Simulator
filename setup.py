import os
import sys
import subprocess
from pathlib import Path

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """Extension class for CMake-based builds"""
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """Custom build command that runs CMake"""

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Create build directory
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        cmake_args = [
            f'-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}',
            f'-DPYTHON_EXECUTABLE={sys.executable}',
            '-DCMAKE_BUILD_TYPE=Release',
        ]

        build_args = ['--config', 'Release']

        # Run CMake (use system cmake explicitly)
        cmake_cmd = '/usr/bin/cmake' if os.path.exists('/usr/bin/cmake') else 'cmake'
        subprocess.check_call(
            [cmake_cmd, ext.sourcedir] + cmake_args,
            cwd=self.build_temp
        )

        # Build
        subprocess.check_call(
            [cmake_cmd, '--build', '.'] + build_args,
            cwd=self.build_temp
        )


setup(
    name='physics-engine',
    version='0.1.0',
    author='Sahand Sadeghi',
    description='A 2D rigid body physics simulator in C++ with Python bindings',
    long_description=open('README.md').read() if os.path.exists('README.md') else '',
    long_description_content_type='text/markdown',
    packages=find_packages(where='python'),
    package_dir={'': 'python'},
    ext_modules=[CMakeExtension('physics_engine_core', sourcedir='cpp')],
    cmdclass={'build_ext': CMakeBuild},
    install_requires=[
        'pybind11>=2.10.0',
        'pygame>=2.5.0',
        'numpy>=1.21.0',
    ],
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Topic :: Scientific/Engineering :: Physics',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
)
