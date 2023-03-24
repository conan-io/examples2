#!/bin/bash

echo "- AutotoolsToolchain: The toolchain generator for Autotools -"

set -ex

conan install .
source conanbuild.sh
aclocal
automake --add-missing
autoconf
./configure
make
output="$(./string_formatter)"
assert_eq "Conan - The C++ Package Manager!" "$output" "not equivalent!"
