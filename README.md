GLADYS
======

*The Graph Library for Autonomous and DYnamic Systems*

[![Build Status](https://travis-ci.org/pierriko/gladys.png?branch=master)]
(https://travis-ci.org/pierriko/gladys)
[![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.17205.svg)]
(http://dx.doi.org/10.5281/zenodo.17205)

Provide an API for high level robotic planners from Georeferenced data.

    +------+    +--------+    +--------------+
    |      |    |        |    |              |
    | GDAL |--->| gladys |--->| Boost::Graph |
    |      |    |        |    |              |
    +------+    +--------+    +--------------+


* http://gdal.org
* http://boost.org/libs/graph
* http://www.openrobots.org/wiki
* http://trac.laas.fr/git/gladys


INSTALL
-------

First, install [`gdalwrap`](https://github.com/pierriko/gdalwrap#install), then

    git clone http://trac.laas.fr/git/gladys && cd gladys
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/devel ..
    make -j8 && make test && make install
    # for verbose test:
    ./test/test_gladys --log_level=test_suite

*cf.* [.travis.yml](.travis.yml)


CONTRIBUTE
----------

Code is available on GitHub at https://github.com/pierriko/gladys

Feel free to fork, pull request or submit issues to improve the project!

* https://github.com/pierriko/gladys/fork
* https://github.com/pierriko/gladys/issues
* https://github.com/pierriko/gladys/pulls
* https://help.github.com/articles/fork-a-repo
* https://help.github.com/articles/using-pull-requests

### STYLE

Please configure your editor to insert 4 spaces instead of TABs, maximum line
length to 79, `lower_case_with_underscores` instead of `CamelCase`. Most of the
rules are taken from [Python PEP8](http://www.python.org/dev/peps/pep-0008/)

Other ideas can be found in Google Guides:
[Python](http://google-styleguide.googlecode.com/svn/trunk/pyguide.html),
[C++](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml).


LICENSE
-------

[BSD 2-Clause](http://opensource.org/licenses/BSD-2-Clause)

Copyright © 2013-2015 CNRS-LAAS
