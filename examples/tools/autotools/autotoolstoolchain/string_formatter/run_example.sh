#!/bin/bash

echo "- AutotoolsToolchain: The toolchain generator for Autotools -"

set -ex

conan install -r conancenter . --build=missing ${PROFILE_ARGS}
source conanbuild.sh
# Remove : from the path when building on Windows
PKG_CONFIG_PATH="${PKG_CONFIG_PATH//:}"

aclocal
automake --add-missing
autoconf
./configure
make

source deactivate_conanbuild.sh

source conanrun.sh

output=$(./string_formatter)

if [[ "$output" != 'Conan - The C++ Package Manager!' ]]; then
    echo "ERROR: The String Formatter output does not match with the expected value: 'Conan - The C++ Package Manager!'"
    exit 1
fi

echo 'AutotoolsToolchain example has been executed with SUCCESS!'
exit 0
