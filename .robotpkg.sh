#!/bin/sh

__NEW_VER=0.1.5

__PKGNAME=gladys
__IS_WIP_=wip/
__RPKROOT=$HOME/robotpkg

# need to run inside the project folder
# with the previous PACKAGE_VERSION in CMakeLists.txt

# do not edit following (supposed to be smart)

__VERSION=$(grep "PACKAGE_VERSION" CMakeLists.txt | cut -d\" -f2)
__DIRNAME=$__PKGNAME-$__NEW_VER
__ARCHIVE=$__DIRNAME.tar.gz

__SHORTLG=$(mktemp)
echo "Changes since v$__VERSION:" > $__SHORTLG
echo "" >> $__SHORTLG
git shortlog v$__VERSION..HEAD >> $__SHORTLG

git commit . -m"Bump to v$__NEW_VER"
git tag v$__NEW_VER -F $__SHORTLG

git archive --format=tar --prefix=$__DIRNAME/ v$__NEW_VER | gzip > $__RPKROOT/distfiles/$__ARCHIVE
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

rm $__SHORTLG
