#!/bin/sh

# Install nlopt from source since Ubuntu 12.04 does not provide debian package for nlopt
curl -o nlopt-2.4.1.tar.gz http://ab-initio.mit.edu/nlopt/nlopt-2.4.1.tar.gz
tar -xf nlopt-2.4.1.tar.gz
cd nlopt-2.4.1/
sh autogen.sh &>/dev/null  # mute the output
make CPPFLAGS='-fPIC' && sudo make install &>/dev/null  # mute the output

# Install eigen-3.2.1 (for unsupported/Eigen/Splines)
wget --quiet -O libeigen3-dev_3.2.1-1~precise1_all.deb http://packages.yade-dem.org/precise/libeigen3-dev_3.2.1-1~precise1_all.deb
sudo dpkg -i libeigen3-dev_3.2.1-1~precise1_all.deb
