"""
Plugin to sign/verify Conan packages with OpenSSL.

You will need to have ``openssl`` installed at the system level and available in your ``PATH``.

To use this plugin, first generate a compatible keypair:

    $ openssl genpkey -algorithm RSA -out private_key.pem -pkeyopt rsa_keygen_bits:2048

And extract the public key:

    $ openssl pkey -in private_key.pem -pubout -out public_key.pem

The private_key.pem and public_key.pem files should be placed inside a folder named with the the provider's name
('my-organization' for this example). The 'my-organization' folder should be next to this plugins' file sign.py
(inside the CONAN_HOME/extensions/plugins/sign folder).

SECURITY NOTE:
    This example stores a private key next to the plugin for simplicity. **Do not do this in production**.
    Instead, load the signing key from environment variables or a secret manager, or delegate signing to a remote signing service.
    **Always keep the private key out of the Conan cache and out of source control**.
"""

import os
import json
import subprocess

from conan.api.output import ConanOutput
from conan.errors import ConanException


def _run_command(command):
    ConanOutput().info(f"Running command: {' '.join(command)}")
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,  # returns strings instead of bytes
        check=False  # we'll manually handle error checking
    )

    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode, result.args, output=result.stdout, stderr=result.stderr
        )


def sign(ref, artifacts_folder, signature_folder, **kwargs):
    provider = "my-organization"  # This maps to the folder containing the signing keys (for simplicity)
    manifest_filepath = os.path.join(signature_folder, "pkgsign-manifest.json")
    signature_filename = "pkgsign-manifest.json.sig"
    signature_filepath = os.path.join(signature_folder, signature_filename)
    if os.path.isfile(signature_filepath):
        ConanOutput().warning(f"Package {ref.repr_notime()} was already signed")

    privkey_filepath = os.path.join(os.path.dirname(__file__), provider, "private_key.pem")
    # openssl dgst -sha256 -sign private_key.pem -out document.sig document.txt
    openssl_sign_cmd = [
        "openssl",
        "dgst",
        "-sha256",
        "-sign", privkey_filepath,
        "-out", signature_filepath,
        manifest_filepath
    ]
    try:
        _run_command(openssl_sign_cmd)
        ConanOutput().success(f"Package signed for reference {ref}")
    except Exception as exc:
        raise ConanException(f"Error signing artifact: {exc}")
    return [{"method": "openssl-dgst",
             "provider": provider,
             "sign_artifacts": {
                "manifest": "pkgsign-manifest.json",
                "signature": signature_filename}}]


def verify(ref, artifacts_folder, signature_folder, files, **kwargs):
    signatures_path = os.path.join(signature_folder, "pkgsign-signatures.json")
    try:
        with open(signatures_path, "r", encoding="utf-8") as f:
            signatures = json.loads(f.read()).get("signatures")
    except Exception:
        ConanOutput().warning("Could not verify unsigned package")
        return

    for signature in signatures:
        signature_filename = signature.get("sign_artifacts").get("signature")
        signature_filepath = os.path.join(signature_folder, signature_filename)
        if not os.path.isfile(signature_filepath):
            raise ConanException(f"Signature file does not exist at {signature_filepath}")

        # The provider is useful to choose the correct public key to verify packages with
        provider = signature.get("provider")
        pubkey_filepath = os.path.join(os.path.dirname(__file__), provider, "public_key.pem")
        if not os.path.isfile(pubkey_filepath):
            raise ConanException(f"Public key not found for provider '{provider}'")

        manifest_filepath = os.path.join(signature_folder, "pkgsign-manifest.json")
        signature_method = signature.get("method")
        if signature_method == "openssl-dgst":
            # openssl dgst -sha256 -verify public_key.pem -signature document.sig document.txt
            openssl_verify_cmd = [
                "openssl",
                "dgst",
                "-sha256",
                "-verify", pubkey_filepath,
                "-signature", signature_filepath,
                manifest_filepath,
            ]
            try:
                _run_command(openssl_verify_cmd)
                ConanOutput().success(f"Package verified for reference {ref}")
            except Exception as exc:
                raise ConanException(f"Error verifying signature {signature_filepath}: {exc}")
        else:
            raise ConanException(f"Sign method {signature_method} not supported. Cannot verify package")
