#!/bin/bash

set -e
set -x

echo "Setup settings user"
cp -f settings_user.yml $(conan config home)

echo "Conan Examples 2: Compiler Sanitizers - Index Out of Bounds"

conan export index_out_of_bounds/
conan install --requires=index_out_of_bounds/0.1.0 -pr profiles/asan -of index_out_of_bounds/install --build=missing -c tools.compilation:verbosity=verbose
source index_out_of_bounds/install/conanrun.sh
index_out_of_bounds || true
. index_out_of_bounds/install/deactivate_conanrun.sh

echo "Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow"

conan export signed_integer_overflow/
conan install --requires=signed_integer_overflow/0.1.0 -pr profiles/asan_ubsan -of signed_integer_overflow/install --build=missing -c tools.compilation:verbosity=verbose
source signed_integer_overflow/install/conanrun.sh
signed_integer_overflow || true
. signed_integer_overflow/install/deactivate_conanrun.sh
