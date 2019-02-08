@echo off

set "CONDA_ROOT=C:\Miniconda3-x64"
set PATH=%CONDA_ROOT%;%CONDA_ROOT%\\scripts;%CONDA_ROOT%\\Library\\bin;%PATH%;C:\\Program Files (x86)\\CMake\\bin
conda config --set always_yes yes
conda config --add channels conda-forge
conda config --add channels anaconda
conda update -q conda
conda config --set auto_update_conda no
conda update -q --all
conda info
python -c "import sys; print(sys.version)"
python -c "import sys; print(sys.executable)"
python -c "import sys; print(sys.prefix)"
call "%CONDA_ROOT%\Scripts\activate.bat" base
conda install geotiff laszip nitro curl gdal pcl cmake eigen ninja libgdal geos zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 oracle-instantclient numpy-base
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64


SET PDAL_INSTALL_PREFIX="C:/projects/pdal/install"
SET PDAL_PLUGIN_INSTALL_PATH="C:/projects/pdal/build/bin"
SET PDAL_BUILD_TESTS=ON
SET CMAKE_VERBOSE_MAKEFILE=OFF
SET CMAKE_BUILD_TYPE=RelWithDebInfo

mkdir build
pushd build

REM activate base
set ORACLE_HOME=%CONDA_ROOT%

cmake -G "Ninja" ^
    -DCMAKE_BUILD_TYPE=%CMAKE_BUILD_TYPE% ^
	-DPDAL_PLUGIN_INSTALL_PATH:FILEPATH=%PDAL_PLUGIN_INSTALL_PATH% ^
    -DWITH_TESTS=%PDAL_BUILD_TESTS% ^
    -DCMAKE_VERBOSE_MAKEFILE=%CMAKE_VERBOSE_MAKEFILE% ^
    -DCMAKE_LIBRARY_PATH:FILEPATH="=%CONDA_ROOT%/Library/lib" ^
    -DCMAKE_INCLUDE_PATH:FILEPATH="%CONDA_ROOT%/Library/include" ^
    -DBUILD_PLUGIN_CPD=OFF ^
    -DBUILD_PLUGIN_GREYHOUND=ON ^
    -DBUILD_PLUGIN_ICEBRIDGE=ON ^
    -DBUILD_PLUGIN_MRSID=OFF ^
    -DBUILD_PLUGIN_NITF=ON ^
    -DBUILD_PLUGIN_PCL=ON ^
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
    -DBUILD_PLUGIN_OCI=ON ^
    -DBUILD_PLUGIN_SQLITE=ON ^
    -DBUILD_PLUGIN_I3S=ON ^
    -DBUILD_PLUGIN_RIVLIB=OFF ^
    -DBUILD_PLUGIN_PYTHON=ON ^
    -DENABLE_CTEST=OFF ^
    -DWITH_LAZPERF=ON ^
    -DWITH_LZMA=ON ^
    -DLIBLZMA_LIBRARY:FILEPATH=%CONDA_ROOT%\Library\lib\liblzma.lib ^
    -DWITH_LASZIP=ON ^
    -DORACLE_INCLUDE_DIR=%CONDA_ROOT%/include ^
    -DORACLE_LIBRARY=%CONDA_ROOT%/libs/oci.lib ^
    -DLazperf_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DHDF5_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DPCL_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DLazperf_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DWITH_ZLIB=ON ^
    -Dgtest_force_shared_crt=ON ^
    -DBUILD_PGPOINTCLOUD_TESTS=OFF ^
    -DBUILD_SQLITE_TESTS=OFF ^
    -DBUILD_I3S_TESTS=OFF ^
    -DBUILD_OCI_TESTS=OFF ^
    ..

popd

