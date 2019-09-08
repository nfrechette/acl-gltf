[![CLA assistant](https://cla-assistant.io/readme/badge/nfrechette/acl-gltf)](https://cla-assistant.io/nfrechette/acl-gltf)
[![Build Status](https://travis-ci.org/nfrechette/acl-gltf.svg?branch=develop)](https://travis-ci.org/nfrechette/acl-gltf)
[![GitHub release](https://img.shields.io/github/release/nfrechette/acl-gltf.svg)](https://github.com/nfrechette/acl-gltf/releases)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/nfrechette/acl-gltf/master/LICENSE)

# Animation Compression Library glTF toolkit

This toolkit contains a set of tools to convert raw glTF animations from/to compressed glTF animations by using the [Animation Compression Library](https://github.com/nfrechette/acl). In time, a proper glTF extension will be standardized.

## Supported platforms

*  Windows (VS2015, VS2017, VS2019) x86 and x64
*  Linux (gcc5, gcc6, gcc7, gcc8, clang4, clang5, clang6) x86 and x64
*  OS X (Xcode 8.3, Xcode 9.4, Xcode 10.1) x86 and x64

The above supported platform list is only what is tested every release but if it compiles, it should work just fine.

Runtime decompression can be implemented in C++ with [ACL](https://github.com/nfrechette/acl) and in Javascript with [acl-js](https://github.com/nfrechette/acl-js).

## Getting started

The toolkit executables currently need to be built by hand with CMake. However, if you wish to run the unit tests or to contribute to acl-gltf head on over to the [development setup](./docs/development_setup.md) section in order to setup your environment and make sure to check out the [contributing guidelines](CONTRIBUTING.md).

## External dependencies

You don't need anything else to get started: everything is self contained.
See [here](./external) for details.

## License, copyright, and code of conduct

This project uses the [MIT license](LICENSE).

Copyright (c) 2019 Nicholas Frechette & contributors

Please note that this project is released with a [Contributor Code of Conduct](CODE_OF_CONDUCT.md). By participating in this project you agree to abide by its terms.
