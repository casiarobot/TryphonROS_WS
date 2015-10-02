1.1 Install TooN

Get the latest version of TooN from the website (http://www.edwardrosten.com/cvd/toon.html). MCPTAM was developed with TooN v2.1 but also works with TooN v2.2. Installation is the usual configure, make, make install. There isn't really anything to make as it's just a bunch of headers. Follow the installation instructions on the target website.

$ ./configure
$ make
$ sudo make install
1.2 Install libCVD

Get the latest version of libCVD from the website (http://www.edwardrosten.com/cvd/). MCPTAM was developed with release 20121025. Installation is the usual procedure of configure, make, make install. Follow the installation instructions on the target website. The following options are recommended for configure:

$ export CXXFLAGS=-D_REENTRANT
$ ./configure --without-ffmpeg
$ make
$ sudo make install
1.3 Install GVars3

Get the latest version of GVars3 from the website (http://www.edwardrosten.com/cvd/gvars3.html). MCPTAM was developed with version 3.0. Installation is the usual procedure of configure, make, make install. Follow the installation instructions on the target website. The following options are recommended for configure:

$ ./configure --disable-widgets
$ make
$ sudo make install
