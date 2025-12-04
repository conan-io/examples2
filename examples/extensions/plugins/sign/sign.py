"""
Plugin to sign/verify Conan packages with OpenSSL.

Requirements: The following executables should be installed and in the PATH.
    - openssl

To use this sigstore plugins, first generate a compatible keypair and define the environment variables for the keys:

    $ openssl genpkey -algorithm RSA -out private_key.pem -pkeyopt rsa_keygen_bits:2048

And extract the public key:

    $ openssl pkey -in private_key.pem -pubout -out public_key.pem

The private_key.pem and public_key.pem files should be placed next to this plugins's file sign.py
(inside the CONAN_HOME/extensions/plugins/sing folder).
"""


import os
import subprocess
from conan.api.output import ConanOutput
from conan.errors import ConanException
from conan.tools.pkg_signing.plugin import (create_summary_content, get_summary_file_path,
                                            load_summary, save_summary)


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
    c = create_summary_content(artifacts_folder)
    c["method"] = "openssl-dgst"
    c["provider"] = "conan-client"
    save_summary(signature_folder, c)

    # openssl dgst -sha256 -sign private_key.pem -out document.sig document.txt
    summary_filepath = get_summary_file_path(signature_folder)
    signature_filepath = f"{summary_filepath}.sig"
    if os.path.isfile(signature_filepath):
        ConanOutput().warning(f"Package {ref.repr_notime()} was already signed")
    privkey_filepath = os.path.join(os.path.dirname(__file__), "private_key.pem")
    openssl_sign_cmd = [
        "openssl",
        "dgst",
        "-sha256",
        "-sign", privkey_filepath,
        "-out", signature_filepath,
        summary_filepath,
    ]
    try:
        _run_command(openssl_sign_cmd)
    except Exception as exc:
        raise ConanException(f"Error signing artifact {summary_filepath}: {exc}")


def verify(ref, artifacts_folder, signature_folder, files, **kwargs):
    summary_filepath = get_summary_file_path(signature_folder)
    signature_filepath = f"{summary_filepath}.sig"
    pubkey_filepath = os.path.join(os.path.dirname(__file__), "public_key.pem")
    if not os.path.isfile(signature_filepath):
        raise ConanException("Signature file does not exist")

    summary = load_summary(signature_folder)
    # The provider is useful to choose the correct public key to verify packages with
    provider = summary.get("provider")
    if provider != "conan-client":
        raise ConanException(f"The provider does not match (conan-client [expected] != {provider} [actual])."
                             f"Cannot get a public key to verify the package")

    method = summary.get("method")
    if method == "openssl-dgst":
        # openssl dgst -sha256 -verify public_key.pem -signature document.sig document.txt
        openssl_verify_cmd = [
            "openssl",
            "dgst",
            "-sha256",
            "-verify", pubkey_filepath,
            "-signature", signature_filepath,
            summary_filepath,
        ]
        try:
            _run_command(openssl_verify_cmd)
        except Exception as exc:
            raise ConanException(f"Error verifying signature {signature_filepath}: {exc}")
    else:
        raise ConanException(f"Sign method {method} not supported. Cannot verify package")
