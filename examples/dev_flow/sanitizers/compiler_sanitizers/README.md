# Compiler Sanitizers Example

This example follows the documented page https://docs.conan.io/2/examples/dev_flow/sanitizers/compiler_sanitizers.

## Examples

Here are some examples of using compiler sanitizers with Conan.

### Signed Integer Overflow

This example demonstrates how to detect signed integer overflow using compiler sanitizers. The provided C++ code intentionally causes a signed integer overflow, which can be detected when running the program with the appropriate sanitizer flags.

It explores the [Undefined Behavior Sanitizer](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html), **ONLY** available in Clang and GCC; MSVC does not support it (yet).

In order to try the example, you may run the following commands:

```
conan create signed_integer_overflow/ -pr profiles/asan_ubsan
conan install --requires=signed_integer_overflow/0.1.0 -pr profiles/asan_ubsan -of install
source install/conanrun.sh
signed_integer_overflow
```
It's expected to observe a runtime error indicating a signed integer overflow has occurred:

```
Address sanitizer not enabled
/home/conan/.conan2/p/b/signe3b8ad6d59f30b/b/main.cpp:13:9: runtime error: signed integer overflow: 2147483647 + 1 cannot be represented in type 'int'
SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior /home/conan/.conan2/p/b/signe3b8ad6d59f30b/b/main.cpp:13:9
```

### Index Out of Bounds

This example demonstrates how to detect out-of-bounds memory access using compiler sanitizers. The provided C++ code intentionally accesses an out-of-bounds index in an array, which can be detected when running the program with the appropriate sanitizer flags.

It explores the [Address Sanitizer](https://clang.llvm.org/docs/AddressSanitizer.html), available in Clang, GCC and MSVC.

In order to try the example, you may run the following commands:

```
conan create index_out_of_bounds/ -pr profiles/asan
conan install --requires=index_out_of_bounds/0.1.0 -pr profiles/asan -of install
source install/conanrun.sh
index_out_of_bounds
```

It's expected to observe a runtime error indicating an out-of-bounds memory access has occurred:

```
==357155==ERROR: AddressSanitizer: stack-buffer-overflow on address 0x7ffcddcc40e0 at pc 0x5946a605f2eb bp 0x7ffcddcc3f10 sp 0x7ffcddcc3f00
WRITE of size 4 at 0x7ffcddcc40e0 thread T0
    #0 0x5946a605f2ea in main (/home/conan/.conan2/p/b/index7e914f42d466f/p/bin/index_out_of_bounds+0x12ea)
    #1 0x7722f0c29d8f in __libc_start_call_main ../sysdeps/nptl/libc_start_call_main.h:58
    #2 0x7722f0c29e3f in __libc_start_main_impl ../csu/libc-start.c:392
    #3 0x5946a605f3d4 in _start (/home/conan/.conan2/p/b/index7e914f42d466f/p/bin/index_out_of_bounds+0x13d4)

Address 0x7ffcddcc40e0 is located in stack of thread T0 at offset 448 in frame
    #0 0x5946a605f1ef in main (/home/conan/.conan2/p/b/index7e914f42d466f/p/bin/index_out_of_bounds+0x11ef)

  This frame has 1 object(s):
    [48, 448) 'foo' (line 11) <== Memory access at offset 448 overflows this variable
HINT: this may be a false positive if your program uses some custom stack unwind mechanism, swapcontext or vfork
      (longjmp and C++ exceptions *are* supported)
SUMMARY: AddressSanitizer: stack-buffer-overflow (/home/conan/.conan2/p/b/index7e914f42d466f/p/bin/index_out_of_bounds+0x12ea) in main
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
