#!/bin/bash

echo "- MakeDeps: The Makefile dependencies generator for Make -"

set -ex

# Remove cache
rm -rf build

# Then generate conanbuild.sh
conan install -r conancenter . -of build --build=missing
source build/conanbuild.sh

# Build the example
make

source build/deactivate_conanbuild.sh

# Make dynamic library available on PATH
source build/conanrun.sh

output=$(build/string_digest_hex)

expected_str='[9d9bad5e773ca301b2a2977843ce6fb5] To be a Cimmerian warrior, you must have both cunning and balance as well as speed and strength.'

if [[ $expected_str -eq $output ]]; then
    echo "ERROR: The String Formatter output does not match with the expected value: '$(expected_str)'"
    exit 1
fi

echo 'MakeDeps example has been executed with SUCCESS!'
exit 0
