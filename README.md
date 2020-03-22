[![CLA assistant](https://cla-assistant.io/readme/badge/nfrechette/acl-gltf)](https://cla-assistant.io/nfrechette/acl-gltf)
[![Build status](https://ci.appveyor.com/api/projects/status/2h28i0j85nkq2e1e/branch/develop?svg=true)](https://ci.appveyor.com/project/nfrechette/acl-gltf)
[![Build Status](https://travis-ci.com/nfrechette/acl-gltf.svg?branch=develop)](https://travis-ci.com/nfrechette/acl-gltf)
[![Sonar Status](https://sonarcloud.io/api/project_badges/measure?project=nfrechette_acl-gltf&metric=alert_status)](https://sonarcloud.io/dashboard?id=nfrechette_acl-gltf)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/nfrechette/acl-gltf/master/LICENSE)
[![Discord](https://img.shields.io/discord/691048241864769647?label=discord)](https://discord.gg/UERt4bS)

# Animation Compression Library glTF toolkit

This toolkit contains a tool to convert compress and decompress glTF animations by using the [Animation Compression Library](https://github.com/nfrechette/acl). In time, a proper glTF extension will hopefully be standardized.

For the time being, `acl-gltf` only serves as a reference and toy. Sadly, glTF is not a suitable file format for a pipeline. It is not possible for a standalone tool to safely process a glTF file without knowing about and supporting every possible extension used in the wild. As such, `acl-gltf` leaves a lot of unused metadata in its output and no raw data is freed to ensure maximum safety.

===> **THIS TOOL IS NOT SUITABLE FOR PRODUCTION USE!** <===

Until an ACL extension is standardized, it is recommended to use this tool as inspiration to see how to compress glTF animations. As such, it is merely a proof of concept.

## Supported platforms

*  Windows (VS2015, VS2017, VS2019) x86 and x64
*  Windows VS2019 with clang8 x86 and x64
*  Linux (gcc5, gcc6, gcc7, gcc8, gcc9) x86 and x64
*  Linux (clang4, clang5, clang6, clang7, clang8, clang9) x86 and x64
*  OS X (Xcode 8.3, 9.4, 10.3) x86 and x64
*  OS X (Xcode 11.2) x64

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
