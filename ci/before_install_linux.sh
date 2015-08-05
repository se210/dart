before_install() {
  cd /tmp

  # Install nlopt from source since Ubuntu 12.04 does not provide debian package for nlopt
  curl -o nlopt-2.4.1.tar.gz http://ab-initio.mit.edu/nlopt/nlopt-2.4.1.tar.gz
  tar -xf nlopt-2.4.1.tar.gz
  cd nlopt-2.4.1/
  sh autogen.sh &>/dev/null  # mute the output
  make CPPFLAGS='-fPIC' && sudo make install &>/dev/null  # mute the output
}

# Install eigen-3.2.1 (for unsupported/Eigen/Splines)
wget --quiet -O libeigen3-dev_3.2.1-1~precise1_all.deb http://packages.yade-dem.org/precise/libeigen3-dev_3.2.1-1~precise1_all.deb
sudo dpkg -i libeigen3-dev_3.2.1-1~precise1_all.deb

# Install boost 1.55 compatible with C++11
wget --quiet -O boost_1_55_0.tar.gz http://sourceforge.net/projects/boost/files/boost/1.55.0/boost_1_55_0.tar.gz/download
tar -xzf boost_1_55_0.tar.gz
cd boost_1_55_0/
sudo apt-get -qq update
sudo apt-get -qq install build-essential g++ python-dev autotools-dev libicu-dev build-essential libbz2-dev 
./bootstrap.sh --prefix=/usr/local &>/dev/null  # mute the output
n=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
sudo ./b2 --with=all -j $n install &>/dev/null  # mute the output
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/local.conf'
sudo ldconfig

sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo add-apt-repository --yes ppa:fcl-debs/ppa
sudo add-apt-repository --yes ppa:dartsim/ppa
sudo apt-get -qq update

(before_install)

