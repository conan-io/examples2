# Compiler Sanitizers Example

This example follows the documented page https://docs.conan.io/2/security/sanitizers.html about using compiler sanitizers with Conan.

## Examples

Here are some examples of using compiler sanitizers with Conan.

### Configuring Custom Settings

Before trying to build using the profiles prepared to work with sanitizers, you may want to configure some custom settings in your Conan home.
It's not needed to modify the `settings.yml` file, instead, you can install a custom settings using [settings_user.yml](https://docs.conan.io/2/reference/config_files/settings.html#settings-user-yml)

```
cp settings_user.yml $(conan config home)
```

This setting allows you to customize the behavior of the sanitizers, enabling or disabling specific checks as needed.

### Signed Integer Overflow

This example demonstrates how to detect signed integer overflow using compiler sanitizers. The provided C++ code intentionally causes a signed integer overflow, which can be detected when running the program with the appropriate sanitizer flags.

It explores the [Undefined Behavior Sanitizer](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html), **ONLY** available in Clang and GCC; MSVC does not support it (yet).

In order to try the example, you may run the following commands:

```
conan create signed_integer_overflow/ -pr profiles/asan_ubsan
conan build signed_integer_overflow/ --version=0.1.0 -pr profiles/asan_ubsan -of install
signed_integer_overflow/build/Debug/signed_integer_overflow
```
It's expected to observe a runtime error indicating a signed integer overflow has occurred:

```
Address sanitizer not enabled
/home/conan/examples2/security/sanitizers/signed_integer_overflow/main.cpp:13:9: runtime error: signed integer overflow: 2147483647 + 1 cannot be represented in type 'int'
SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior /home/conan/examples2/security/sanitizers/signed_integer_overflow/main.cpp:13:9
```

### Index Out of Bounds

This example demonstrates how to detect out-of-bounds memory access using compiler sanitizers. The provided C++ code intentionally accesses an out-of-bounds index in an array, which can be detected when running the program with the appropriate sanitizer flags.

It explores the [Address Sanitizer](https://clang.llvm.org/docs/AddressSanitizer.html), available in Clang, GCC and MSVC.

In order to try the example, you may run the following commands:

```
conan create index_out_of_bounds/ -pr profiles/asan
conan build index_out_of_bounds/ --version=0.1.0 -pr profiles/asan -of install
index_out_of_bounds/build/Debug/index_out_of_bounds
```

It's expected to observe a runtime error indicating an out-of-bounds memory access has occurred:

```
==357155==ERROR: AddressSanitizer: stack-buffer-overflow on address 0x7ffcddcc40e0 at pc 0x5946a605f2eb bp 0x7ffcddcc3f10 sp 0x7ffcddcc3f00
WRITE of size 4 at 0x7ffcddcc40e0 thread T0
    #0 0x5946a605f2ea in main (/home/conan/examples2/security/sanitizers/index_out_of_bounds/build/Debug/index_out_of_bounds)
    #1 0x7722f0c29d8f in __libc_start_call_main ../sysdeps/nptl/libc_start_call_main.h:58
    #2 0x7722f0c29e3f in __libc_start_main_impl ../csu/libc-start.c:392
    #3 0x5946a605f3d4 in _start (/home/conan/examples2/security/sanitizers/index_out_of_bounds/build/Debug/index_out_of_bounds+0x13d4)

Address 0x7ffcddcc40e0 is located in stack of thread T0 at offset 448 in frame
    #0 0x5946a605f1ef in main (/home/conan/examples2/security/sanitizers/index_out_of_bounds/build/Debug/index_out_of_bounds+0x11ef)

  This frame has 1 object(s):
    [48, 448) 'foo' (line 11) <== Memory access at offset 448 overflows this variable
HINT: this may be a false positive if your program uses some custom stack unwind mechanism, swapcontext or vfork
      (longjmp and C++ exceptions *are* supported)
SUMMARY: AddressSanitizer: stack-buffer-overflow (/home/conan/examples2/security/sanitizers/index_out_of_bounds/build/Debug/index_out_of_bounds+0x12ea) in main
```

## Customizing Sanitizers

### Using Environment Variables

The `ASAN_OPTIONS` and `UBSAN_OPTIONS` environment variables can be used to customize the behavior of AddressSanitizer and UndefinedBehaviorSanitizer, respectively. For example, you can set the `ASAN_OPTIONS` variable to control the reporting format, enable or disable specific checks, and more.

To set these environment variables, you can use the `export` command in your terminal before running your program:

```bash
export ASAN_OPTIONS=detect_leaks=1:log_path=asan.log
export UBSAN_OPTIONS=print_stacktrace=1
```

This will enable leak detection for AddressSanitizer and print stack traces for UndefinedBehaviorSanitizer.

For more advanced configurations, you can refer to the [Clang AddressSanitizer](https://github.com/google/sanitizers/wiki/addresssanitizerflags#run-time-flags) and [MSVC AddressSanitizer](https://learn.microsoft.com/en-us/cpp/sanitizers/asan?view=msvc-170#differences) documentation.
