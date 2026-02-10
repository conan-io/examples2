"""
Plugin to sign/verify Conan packages with OpenSSL.

Requirements: The following executables should be installed and in the PATH.
    - openssl

To use this sigstore plugins, first generate a compatible keypair and define the environment variables for the keys:

    $ openssl genpkey -algorithm RSA -out private_key.pem -pkeyopt rsa_keygen_bits:2048

And extract the public key:

    $ openssl pkey -in private_key.pem -pubout -out public_key.pem

The private_key.pem and public_key.pem files should be placed inside a folder named with the the provider's name
('my-organization' for this example). The 'my-organization' folder should be next to this plugins's file sign.py
(inside the CONAN_HOME/extensions/plugins/sign folder).
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
    signatures = os.path.join(signature_folder, "pkgsign-signatures.json")
    signature = json.loads(open(signatures).read()).get("signatures")[0]
    signature_filename = signature.get("sign_artifacts").get("signature")
    signature_filepath = os.path.join(signature_folder, signature_filename)
    if not os.path.isfile(signature_filepath):
        raise ConanException("Signature file does not exist")

    # The provider is useful to choose the correct public key to verify packages with
    expected_provider = "my-organization"
    signature_provider = signature.get("provider")
    if signature_provider != expected_provider:
        raise ConanException(f"The provider does not match ({expected_provider} [expected] != {signature_provider} "
                              "[actual]). Cannot get a public key to verify the package")
    pubkey_filepath = os.path.join(os.path.dirname(__file__), expected_provider, "public_key.pem")

    manifest_filepath =os.path.join(signature_folder, "pkgsign-manifest.json")
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
