# Example script for compiling

TOPDIR=$(realpath $(dirname $0))

echo "cleaning $TOPDIR/blossom5-v2.05.src"
cd $TOPDIR/blossom5-v2.05.src
make clean

echo "cleaning $TOPDIR/src"
cd $TOPDIR/src
make clean

echo "removing $TOPDIR/bin"
rm -rd $TOPDIR/bin

echo "done."
exit 0

