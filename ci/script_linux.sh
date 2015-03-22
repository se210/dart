make
sudo ldconfig --verbose # So the test executeables can detect libtinyxml2
if [ $BUILD_CORE_ONLY = OFF ]; then make test; fi
sudo make install
# cd tools/
# ./code_check.sh
# ./abi_check.sh 4.1.0  # Check with DART 4.1.0

# Install dart-example to see DART is installed correctly
cd ~
git clone https://github.com/dartsim/dart-examples
cd dart-examples
if [ "$BUILD_CORE_ONLY" = "ON"  ]; then git checkout -b dart-core-4.3 ; fi
if [ "$BUILD_CORE_ONLY" = "OFF" ]; then git checkout -b dart-4.3      ; fi
(mkdir build; cd build; cmake ..; make)
