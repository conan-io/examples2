@echo off
setlocal enabledelayedexpansion

echo Setup settings user
for /f "usebackq delims=" %%H in (conan config home) do set "CONAN_HOME=%%H"
copy /Y settings_user.yml "%CONAN_HOME%"

echo Conan Examples 2: Compiler Sanitizers - Index Out of Bounds

PUSHD index_out_of_bounds/
conan build . -pr profiles/asan -c tools.compilation:verbosity=verbose
build/Debug/index_out_of_bounds 2>nul || echo Process completed with errors (expected for sanitizer demo)
POPD

echo Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow

PUSHD
conan build . -pr profiles/asan_ubsan -c tools.compilation:verbosity=verbose
build/Debug/signed_integer_overflow 2>nul || echo Process completed with errors (expected for sanitizer demo)
POPD

exit /b 0
