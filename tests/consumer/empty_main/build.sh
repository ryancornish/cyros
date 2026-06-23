set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ "$PWD" != "$SCRIPT_DIR" ]; then
   echo "Error: Script must be run from the same directory it resides in."
   exit 1
fi

rm -rf ./out/

cyros-builder build -p build_tree/profile.toml

mkdir ./out/bin/

g++-15 main.cpp \
   -std=gnu++26 \
   -Iout/empty_main/gcc-basic/include/ \
   -Lout/empty_main/gcc-basic/lib/ \
   -lcyros \
   -o out/bin/main