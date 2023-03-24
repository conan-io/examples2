#!/bin/bash

echo "- AutotoolsToolchain: The toolchain generator for Autotools -"

set -ex

conan install . --build=missing
source conanbuild.sh
aclocal
automake --add-missing
autoconf
./configure
make

output=$(./string_formatter)

if [[ "$output" != 'Conan - The C++ Package Manager!' ]]; then
    echo "ERROR: The String Formatter output does not match with the expected value: 'Conan - The C++ Package Manager!'"
    exit 1
fi

echo 'AutotoolsToolchain example has been executed with SUCCESS!'
exit 0
