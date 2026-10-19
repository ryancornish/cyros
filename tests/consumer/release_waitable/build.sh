# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Ryan Cornish

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ "$PWD" != "$SCRIPT_DIR" ]; then
   echo "Error: Script must be run from the same directory it resides in."
   exit 1
fi

rm -rf ./out/

cyros-builder build --profile profile.toml

mkdir ./out/bin/

g++-16 main.cpp \
   -std=gnu++26 -O2 \
   -Iout/release_waitable/gcc-release/include/ \
   -Lout/release_waitable/gcc-release/lib/ \
   -lcyros \
   -o out/bin/main

echo "Created ./out/bin/main"
