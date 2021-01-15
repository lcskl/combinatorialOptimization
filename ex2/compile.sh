# Example script for compiling

if [[ $# -gt 2 ]] || [[ $# -eq 1 ]] && [[ "$1" != "opt" ]] && [[ "$1" != "debug" ]]; then
  echo "Usage: $0 [opt|debug]"
fi


TOPDIR=$(realpath $(dirname $0))

echo "make blossom5-v2.05 ..."
cd $TOPDIR/blossom5-v2.05.src
make
EXITSTATUS=$?
echo

if [[ $EXITSTATUS == 0 ]]; then
  echo "make executable ..."
  cd $TOPDIR/src
  make $*
  EXITSTATUS=$?
  echo ""
fi

# Create bin dir with link to executable
if [[ $EXITSTATUS == 0 ]]; then
  echo "Creating $TOPDIR/bin/minimum-mean-cycle ..."
  mkdir -p $TOPDIR/bin
  ln -sf $TOPDIR/src/build/minimum-mean-cycle $TOPDIR/bin/
  echo "done."
else
  echo "Compilation failed"
fi

exit $EXITSTATUS

