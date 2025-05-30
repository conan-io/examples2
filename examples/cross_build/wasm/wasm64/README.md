# WASM 32 bit vs 64 bit projet comparison


This basic example aims to show the differences between cross compiling a project with `arch=wasm` (which stands for `wasm32` bits)
and `arch=wasm64` the newly introduced architecture which supports WebAssembly 64-bits.


To compile the project in 32 bits follow this instructions:

```sh
$ conan build . -pr ../profiles/wasm32
$ node build/release-wasm/wasm-alloc.js

Current allocated: 1 GiB
Current allocated: 2 GiB
Current allocated: 3 GiB
Failed after allocating 4064 MiB ~ 3 GiB
```

As we can see, in 32 bits mode, we can allocate up to 4 GB of dynamic memory. This is a limitation of `wasm32` architecture.
I we take a look at the `wasm32` profile, we can see `sMAXIMUM_MEMORY=4GB`. Trying to increase this number will result in a compilation error similar to this one:

```
wasm-ld: error: maximum memory too large, cannot be greater than 4294967296
```

This is the main reason for `wasm64` to exist. Mind than `wasm64` is still under development:
To compile this project in `64 bit` mode, run the following commands:

```sh
$ conan build . -pr ../profiles/wasm64
$ node build/release-wasm64/wasm-alloc.js

Current allocated: 1 GiB
Current allocated: 2 GiB
Current allocated: 3 GiB
Current allocated: 4 GiB
Current allocated: 5 GiB
Current allocated: 6 GiB
Current allocated: 7 GiB
Current allocated: 8 GiB
Current allocated: 9 GiB
Current allocated: 10 GiB
Current allocated: 11 GiB
Current allocated: 12 GiB
Current allocated: 13 GiB
Current allocated: 14 GiB
Current allocated: 15 GiB
Failed after allocating 16352 MiB ~ 15 GiB
```

The difference is notable, the 4 GB limitation does not exist more in a `64 bit` architecture.
The dynamic memory limit could be easily increased by modifying the profile.

