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
   -std=gnu++26 -O0 -g3 \
   -Iout/basic_project_1/gcc-basic/include/ \
   -Lout/basic_project_1/gcc-basic/lib/ \
   -lcyros \
   -lboost_context \
   -o out/bin/main

echo "Created ./out/bin/main"