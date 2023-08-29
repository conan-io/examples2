@echo on

echo "- MakeDeps: The Makefile dependencies generator for Make -"

Rem Remove cache
@rd /S /Q build

Rem Then generate conanbuild.sh
call conan install -r conancenter . -of build --build=missing
call build/conanbuild.bat

Rem Build the example
make

call build/deactivate_conanbuild.bat

Rem Make dynamic library available on PATH
call build/conanrun.sh

call build/string_digest_hex.exe

echo 'MakeDeps example has been executed with SUCCESS!'

exit 0
