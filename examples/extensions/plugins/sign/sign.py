"""
Plugin to sign/verify Conan packages with OpenSSL.

Requirements: The following executables should be installed and in the PATH.
    - openssl

To use this sigstore plugins, first generate a compatible keypair and define the environment variables for the keys:

    $ openssl genpkey -algorithm RSA -out private_key.pem -pkeyopt rsa_keygen_bits:2048

And extract the public key:

    $ openssl pkey -in private_key.pem -pubout -out public_key.pem

The private_key.pem and public_key.pem files should be placed inside a folder named withe the provider's name
('conan-client' for this example). The conan-client folder should be next to this plugins's file sign.py
(inside the CONAN_HOME/extensions/plugins/sing folder).
"""


import os
import subprocess
from conan.api.output import ConanOutput
from conan.errors import ConanException
from conan.tools.pkg_signing.plugin import (get_manifest_filepath, load_manifest,
                                            load_signatures, verify_files_checksums)


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
    provider = "conan-client"  # This maps to the folder containing the signing keys (for simplicity)
    manifest_filepath = get_manifest_filepath(signature_folder)
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
    except Exception as exc:
        raise ConanException(f"Error signing artifact {summary_filepath}: {exc}")
    return [{"method": "openssl-dgst",
             "provider": provider,
             "sign_artifacts": {"signature": signature_filename}}]


def verify(ref, artifacts_folder, signature_folder, files, **kwargs):
    verify_files_checksums(signature_folder, files)

    signature = load_signatures(signature_folder).get("signatures")[0]
    signature_filename = signature.get("sign_artifacts").get("signature")
    signature_filepath = os.path.join(signature_folder, signature_filename)
    if not os.path.isfile(signature_filepath):
        raise ConanException("Signature file does not exist")

    # The provider is useful to choose the correct public key to verify packages with
    expected_provider = "conan-client"
    signature_provider = signature.get("provider")
    if signature_provider != expected_provider:
        raise ConanException(f"The provider does not match ({expected_provider} [expected] != {signature_provider} "
                              "[actual]). Cannot get a public key to verify the package")
    pubkey_filepath = os.path.join(os.path.dirname(__file__), "conan-client", "public_key.pem")

    manifest_filepath = get_manifest_filepath(signature_folder)
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
        except Exception as exc:
            raise ConanException(f"Error verifying signature {signature_filepath}: {exc}")
    else:
        raise ConanException(f"Sign method {signature_method} not supported. Cannot verify package")
