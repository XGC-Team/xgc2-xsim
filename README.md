# xsim

[Maintainer documentation](https://github.com/XGC-Team/xgc2-dev-memory/blob/master/now/experiment-simulation-backends.md#xsim-product-overview-and-startup).

### Source build and staged installation

`.xgc2/scripts/build_source.sh BUILD_DIR INSTALL_ROOT FS150_ASSET_SOURCE` builds
XSim and its GPU sensing library using dependencies already provided by the
caller. It performs no APT installation, Docker operation or publication.
`BUILD_JOBS`, `CMAKE_PREFIX_PATH`, `XGC2_MATH_INCLUDE` and optional
`XGC2_XSIM_PYTHON` select the prepared build environment. Set
`XGC2_SOURCE_TESTS=ON` to build and run the source checks. The Debian packaging
entry uses this same source build, then packages the staged output. GCC 8 links
its separate C++17 filesystem compatibility library.
