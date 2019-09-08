# Getting started

In order to contribute to or build acl-gltf you will first need to setup your environment.

## Setting up your environment

### Windows, Linux, and OS X for x86 and x64

1. Install *CMake 3.2* or higher (*3.14* for Visual Studio 2019, or *3.10* on OS X with *Xcode 10*), *Python 3*, and the proper compiler for your platform.
2. Execute `git submodule update --init` to get the files of thirdparty submodules (e.g. Catch2).
3. Generate the IDE solution with: `python make.py`  
   The solution is generated under `./build`  
   Note that if you do not have CMake in your `PATH`, you should define the `ACL_GLTF_CMAKE_HOME` environment variable to something like `C:\Program Files\CMake`.
4. Build the IDE solution with: `python make.py -build`

On all three platforms, *AVX* support can be enabled by using the `-avx` switch.

## Commit message format

This library uses the [angular.js message format](https://github.com/angular/angular.js/blob/master/DEVELOPERS.md#commits) and it is enforced with commit linting through every pull request.
