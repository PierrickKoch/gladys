#!/bin/sh

__PKGNAME=gladys
__IS_WIP_=wip/
__RPKROOT=$HOME/robotpkg
# need to run inside the project folder
# after updating the PACKAGE_VERSION in CMakeLists.txt
# and commiting it

# do not edit following (supposed to be smart)

__VERSION=$(grep "PACKAGE_VERSION" CMakeLists.txt | cut -d\" -f2)
__DIRNAME=$__PKGNAME-$__VERSION
__ARCHIVE=$__DIRNAME.tar.gz

git tag v$__VERSION
git archive --format=tar --prefix=$__DIRNAME/ v$__VERSION | gzip > $__RPKROOT/distfiles/$__ARCHIVE
cd $__RPKROOT/$__IS_WIP_$__PKGNAME

vi Makefile # edit new version TODO use `sed`

make distinfo
make clean
make deinstall
make update
make print-PLIST
# update PLIST only if changes
test `diff -u0 PLIST PLIST.guess | wc -l` -gt 5 && mv PLIST.guess PLIST
scp $__RPKROOT/distfiles/$__ARCHIVE anna.laas.fr:/usr/local/openrobots/distfiles/$__PKGNAME/
git commit . -m"[$__IS_WIP_$__PKGNAME] Update to $__DIRNAME"
# add changelog TODO `git commit --amend`
# use git log --oneline v0.1.2..v0.1.3
