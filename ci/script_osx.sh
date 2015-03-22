make
if [ $BUILD_CORE_ONLY = OFF ]; then make test; fi
sudo make install
# cd tools/
# ./code_check.sh
# ./abi_check.sh 4.1.0  # Check with DART 4.1.0

# Install dart-example to see DART is installed correctly
cd ~
if [ "$BUILD_CORE_ONLY" = "ON"  ]; then git clone -b dart-core-4 https://github.com/dartsim/dart-examples ; fi
if [ "$BUILD_CORE_ONLY" = "OFF" ]; then git clone -b dart-4 https://github.com/dartsim/dart-examples      ; fi
(cd dart-examples; mkdir build; cd build; cmake ..; make)
