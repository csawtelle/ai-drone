


git clone https://github.com/scanse/sweep-sdk.git

cd libsweep
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig
cd ../..

cd sweeppy
python3 setup.py install --user
