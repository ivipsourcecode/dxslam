#!/bin/bash -e

echo "Configuring and building Thirdparty/cnpy ..."

cd Thirdparty/cnpy
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "configuring and building Thirdparty/fbow"

cd ../../fbow
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


echo "configuring and building Thirdparty/Dbow2"


cd ../../DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf *.tar.xz
cd ..

cd hf-net
tar -xf *.tar.xz
cd ..


echo "Configuring and building DXSLAM ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
