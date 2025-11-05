@echo off
setlocal enabledelayedexpansion

echo Setup settings user
for /f "usebackq delims=" %%H in (conan config home) do set "CONAN_HOME=%%H"
copy /Y settings_user.yml "%CONAN_HOME%"

echo Conan Examples 2: Compiler Sanitizers - Index Out of Bounds

CD index_out_of_bounds/
CALL conan build . -pr ../profiles/msvc_asan -c tools.compilation:verbosity=verbose
CALL build/Debug/index_out_of_bounds 2>nul || echo Process completed with errors (expected for sanitizer demo)
CD ..

echo Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow

CD signed_integer_overflow/
CALL conan build . -pr ../profiles/msvc_asan -c tools.compilation:verbosity=verbose
CALL build/Debug/signed_integer_overflow 2>nul || echo Process completed with errors (expected for sanitizer demo)
CD ..

exit /b 0
