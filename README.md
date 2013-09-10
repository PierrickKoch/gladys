GLADYS
======

*The Graph Library for Autonomous and DYnamic Systems*

[![Build Status](https://travis-ci.org/pierriko/gladys.png?branch=master)]
(https://travis-ci.org/pierriko/gladys)

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

    git clone http://trac.laas.fr/git/gladys && cd gladys
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/devel ..
    make -j8 && make test && make install
    # for verbose test:
    ./test/test_gladys --log_level=test_suite


CONTRIBUTE
----------

Code is available on GitHub at https://github.com/pierriko/gladys

Feel free to fork, pull request or submit issues to improve the project!

* https://github.com/pierriko/gladys/fork
* https://github.com/pierriko/gladys/issues
* https://github.com/pierriko/gladys/pulls
* https://help.github.com/articles/fork-a-repo
* https://help.github.com/articles/using-pull-requests


LICENSE
-------

[BSD 3-Clause](http://opensource.org/licenses/BSD-3-Clause)
