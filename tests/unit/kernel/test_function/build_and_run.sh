set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ "$PWD" != "$SCRIPT_DIR" ]; then
   echo "Error: Script must be run from the same directory it resides in."
   exit 1
fi

rm -rf ./out/

cortos-builder build --profile ../../../../build/profiles/unit_test.toml --config test_function_config.hpp

mkdir ./out/ ./out/bin/

g++-15 test_function.cpp \
   -std=gnu++26 -O0 -g3 \
   -Iout/gcc-basic/include/ \
   -Lout/gcc-basic/lib/ \
   -lcortos \
   -lboost_context \
   -lgtest \
   -lgtest_main \
   -o out/bin/test_function

echo "Created ./out/bin/test_function"
echo "Executing test:"
./out/bin/test_function
