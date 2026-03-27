"""
Example-only plugin to sign and verify Conan packages with Sigstore (cosign).

**Not production-ready** — illustrative code to show how the signing hook can call cosign. Adapt and harden before any real use.

Requires **cosign 3.0.0 or newer** on ``PATH`` (https://github.com/sigstore/cosign/releases).

Generate a key pair (you will be prompted for a password unless you rely on ``COSIGN_PASSWORD``):

    $ cosign generate-key-pair --output-key-prefix signing

Place ``signing.key`` and ``signing.pub`` in a folder named after your provider (``my-organization`` in this
example), next to this file under ``CONAN_HOME/extensions/plugins/sign/``.

For non-interactive signing (CI, automation), set ``COSIGN_PASSWORD`` to the key password (empty string if the
key has no password).

This plugin explicitly disables the usage of Rekor public log by diabling it in the signining and verify processes

SECURITY NOTE:
    This example keeps keys beside the plugin for clarity only. **Do not do this in production** — use a
    secret store, or simmilar, and never commit private keys.
"""

import json
import os
import subprocess

from conan.api.output import ConanOutput
from conan.errors import ConanException

# Stored in pkgsign-signatures.json as ``method``; use a stable, distinctive name across your org/ecosystem.
SIGNING_METHOD = "sigstore"
BUNDLE_FILENAME = "artifact.sigstore.json"


def _signing_config_path():
    return os.path.join(os.path.dirname(__file__), "signing-config.json")


def _run_command(command):
    ConanOutput().info(f"Running command: {' '.join(command)}")
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode, result.args, output=result.stdout, stderr=result.stderr
        )


def sign(ref, artifacts_folder, signature_folder, **kwargs):
    provider = "my-organization"
    manifest_filepath = os.path.join(signature_folder, "pkgsign-manifest.json")
    bundle_filepath = os.path.join(signature_folder, BUNDLE_FILENAME)

    if os.path.isfile(bundle_filepath):
        ConanOutput().warning(f"Package {ref.repr_notime()} was already signed (bundle exists)")

    if "COSIGN_PASSWORD" not in os.environ:
        raise ConanException(
            "COSIGN_PASSWORD is not set. Set it to your cosign key password (use an empty value if the key "
            "has no password) so signing can run non-interactively."
        )

    privkey_filepath = os.path.join(os.path.dirname(__file__), provider, "signing.key")
    if not os.path.isfile(privkey_filepath):
        raise ConanException(f"Private key not found at {privkey_filepath}")

    cosign_sign_cmd = [
        "cosign",
        "sign-blob",
        "--key",
        privkey_filepath,
        "--bundle",
        bundle_filepath,
        "-y",
        f"--signing-config={_signing_config_path()}",
        manifest_filepath,
    ]
    try:
        _run_command(cosign_sign_cmd)
        ConanOutput().success(f"Package signed for reference {ref}")
    except Exception as exc:
        raise ConanException(f"Error signing artifact: {exc}") from exc

    return [
        {
            "method": SIGNING_METHOD,
            "provider": provider,
            "sign_artifacts": {
                "manifest": "pkgsign-manifest.json",
                "bundle": BUNDLE_FILENAME,
            },
        }
    ]


def verify(ref, artifacts_folder, signature_folder, files, **kwargs):
    signatures_path = os.path.join(signature_folder, "pkgsign-signatures.json")
    try:
        with open(signatures_path, "r", encoding="utf-8") as f:
            signatures = json.loads(f.read()).get("signatures")
    except Exception:
        ConanOutput().warning("Could not verify unsigned package")
        return

    for signature in signatures:
        artifacts = signature.get("sign_artifacts") or {}
        manifest_name = artifacts.get("manifest")
        bundle_name = artifacts.get("bundle")
        if not manifest_name or not bundle_name:
            raise ConanException("Signature entry missing manifest or bundle path")

        manifest_filepath = os.path.join(signature_folder, manifest_name)
        bundle_filepath = os.path.join(signature_folder, bundle_name)
        if not os.path.isfile(bundle_filepath):
            raise ConanException(f"Signature bundle not found at {bundle_filepath}")

        provider = signature.get("provider")
        pubkey_filepath = os.path.join(os.path.dirname(__file__), provider, "signing.pub")
        if not os.path.isfile(pubkey_filepath):
            raise ConanException(f"Public key not found for provider '{provider}'")

        signature_method = signature.get("method")
        if signature_method == SIGNING_METHOD:
            cosign_verify_cmd = [
                "cosign",
                "verify-blob",
                "--key",
                pubkey_filepath,
                "--bundle",
                bundle_filepath,
                "--private-infrastructure=true",
                manifest_filepath,
            ]
            try:
                _run_command(cosign_verify_cmd)
                ConanOutput().success(f"Package verified for reference {ref}")
            except Exception as exc:
                raise ConanException(f"Error verifying signature {bundle_filepath}: {exc}") from exc
        else:
            raise ConanException(
                f"Sign method {signature_method!r} not supported. Cannot verify package"
            )
