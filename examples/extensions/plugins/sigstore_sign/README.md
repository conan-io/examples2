
## Package signing plugin example with Sigstore (cosign)

> **This is not a production-ready plugin.** It is a **minimal example** only, to show how you might wire Conan’s package signing hook to cosign. For real deployments, harden key handling, configuration, error handling, and policy before relying on it.

> **_SECURITY NOTE:_**  This example stores a private key next to the plugin for simplicity. **Do not do this in production**.
> Instead, load the signing key from environment variables or a secret manager, or delegate signing to a remote signing service.
> **Always keep the private key out of the Conan cache and out of source control**.

This example implements Conan’s signing hook with [cosign](https://github.com/sigstore/cosign) in **offline** mode (no public Rekor upload), using the smallest amount of code that still demonstrates sign and verify.


### Requirements

- **`cosign`** on your `PATH` (required >= 3.0.0 version). See [releases](https://github.com/sigstore/cosign/releases).
- **`COSIGN_PASSWORD`:** Required environment variable for non-interactive signing (CI, scripts). Set it to the password for your cosign private key. If the key has no password, set it to an empty value (for example `export COSIGN_PASSWORD=` in bash). Cosign reads this variable instead of prompting.

### Signing method name

The plugin records **`method`: `sigstore-cosign`** in signature metadata (`pkgsign-signatures.json`). That string is the convention for this cosign-based backend: keep it stable so verifiers and other tooling can recognize and handle this format alongside other methods (`openssl-dgst`, `gpg`, etc.).

### Try it

1. Copy `sign.py`, `signing-config.json`, and your provider key directory into `CONAN_HOME/extensions/plugins/sign/` (same layout as this folder: `sign.py` and `signing-config.json` at the root of `sign/`, keys under `my-organization/`).
2. Generate keys and move them into `my-organization/` as `signing.key` / `signing.pub`:

   ```bash
   cosign generate-key-pair --output-key-prefix signing
   ```

3. Create and sign a package:

   ```bash
   conan new cmake_lib -d name=hello -d version=1.0
   conan create
   conan cache sign hello/1.0
   conan cache verify hello/1.0
   ```

Packages downloaded from a remote are verified automatically when verification is enabled (e.g. `conan install`), as with any signing plugin.
