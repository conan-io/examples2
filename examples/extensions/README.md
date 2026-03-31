# Conan extensions examples

### [Use custom commands in your Conan CLI](commands)

- Learn how to create custom commands in Conan. [Docs](https://docs.conan.io/2/reference/commands/custom_commands.html)

### [Use custom deployers](deployers)

- Learn how to create a custom deployer in Conan. [Docs](https://docs.conan.io/2/reference/extensions/deployers.html)

### Package signing plugin examples

- Learn how to implement Conan's package signing plugin. [Docs](https://docs.conan.io/2/reference/extensions/package_signing.html)

#### [OpenSSL](plugins/openssl_sign)

- Sign and verify with `openssl dgst` and PEM keys.

#### [Sigstore (cosign)](plugins/sigstore_sign)

- Sign and verify packages with [Sigstore](https://www.sigstore.dev/) using [cosign](https://github.com/sigstore/cosign).
