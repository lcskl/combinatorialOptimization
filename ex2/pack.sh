# Script packing the current directory as a .tar.gz archive file
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <name of archieve without filenending>"
  exit 1
fi

if [[ "$(pwd)" != "$(dirname $(realpath $0))" ]]; then
  echo "$0: Cowardly refusing to pack unknown directory"
  exit 1
fi

tar -czf $1.tar.gz *
exit 0

