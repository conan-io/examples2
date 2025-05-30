# WASM project with bindings and conan dependency


## Build and run

To compile the project:

```sh
$ conan build . -pr:h ../profiles/wasm32 --build=missing
```

To open a WASM webpage locally, most of the browsers will complain due to security reasons as WASM must be loaded asynchronous

The easiest way of opening the generated webpage (should be in `build/release-wasm/wasm_example.html`) is by running a local server.
This can be done via `emrun` command (needs to be downloaded):

```sh
$ emrun --browser <browser_name> build/release-wasm/wasm_example.html   
```

Or using python `http.server` module:

```sh
$ python -m http.server 8080
```
Then, navigating to your build folder and open `wasm_example.html`
