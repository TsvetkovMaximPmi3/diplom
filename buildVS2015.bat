setlocal

rd /Q /S bin
rd /Q /S bld
mkdir bld

@echo off

set SDK_DIR=C:\sdk

pushd bld
cmake -G "Visual Studio 14 2015 Win64" ^
	-D UNISYS_C3D_VERSION=117937 ^
	-D UNISYS_COMPILER_PLATFORM=msvc2015-x86_amd64 ^
	-D CMAKE_RUNTIME_OUTPUT_DIRECTORY=..//bin ^
	-D UNISYS_SDK_DIR=%SDK_DIR% ^
	-D UNISYS_SDK2_DIR=%SDK_DIR% .. 
popd

pause