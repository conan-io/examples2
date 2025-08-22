@echo off
setlocal enabledelayedexpansion

echo Setup settings user
for /f "usebackq delims=" %%H in (conan config home) do set "CONAN_HOME=%%H"
copy /Y settings_user.yml "%CONAN_HOME%"

echo Conan Examples 2: Compiler Sanitizers - Index Out of Bounds

conan export index_out_of_bounds/
conan install --requires=index_out_of_bounds/0.1.0 -pr profiles/asan -of index_out_of_bounds/install --build=missing -c tools.compilation:verbosity=verbose
call index_out_of_bounds\install\conanrun.bat
index_out_of_bounds.exe 2>nul || echo Process completed with errors (expected for sanitizer demo)
call index_out_of_bounds\install\deactivate_conanrun.bat

echo Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow

conan export signed_integer_overflow/
conan install --requires=signed_integer_overflow/0.1.0 -pr profiles/asan_ubsan -of signed_integer_overflow/install --build=missing -c tools.compilation:verbosity=verbose
call signed_integer_overflow\install\conanrun.bat
signed_integer_overflow.exe 2>nul || echo Process completed with errors (expected for sanitizer demo)
call signed_integer_overflow\install\deactivate_conanrun.bat

exit /b 0