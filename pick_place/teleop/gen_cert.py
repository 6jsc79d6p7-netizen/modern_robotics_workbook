"""Self-signed TLS cert for serving teleop over the LAN.

WebXR (and device motion in general) is only exposed in a *secure context*, so
the phone must reach the server over HTTPS/WSS. A self-signed cert works: accept
the browser warning once (Android Chrome). Copied from simple_slam/server; the
Mac's LAN IP goes in as a SAN so the cert matches the URL typed on the phone.
"""
import datetime
import ipaddress
import pathlib
import socket

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

CERTS = pathlib.Path(__file__).resolve().parent / "certs"


def lan_ip() -> str:
    """Best-effort local network IP (the one a phone would reach)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))       # no packets sent; picks outbound iface
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def generate(force: bool = False) -> tuple[pathlib.Path, pathlib.Path]:
    CERTS.mkdir(parents=True, exist_ok=True)
    key_path, cert_path = CERTS / "key.pem", CERTS / "cert.pem"
    if cert_path.exists() and key_path.exists() and not force:
        return cert_path, key_path

    ip = lan_ip()
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "pick-place-teleop")])
    alt = [x509.DNSName("localhost"),
           x509.IPAddress(ipaddress.ip_address("127.0.0.1"))]
    try:
        alt.append(x509.IPAddress(ipaddress.ip_address(ip)))
    except ValueError:
        pass

    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name).issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(days=1))
        .not_valid_after(now + datetime.timedelta(days=825))
        .add_extension(x509.SubjectAlternativeName(alt), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .add_extension(x509.ExtendedKeyUsage(
            [x509.oid.ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .sign(key, hashes.SHA256())
    )
    key_path.write_bytes(key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption()))
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    print(f"[cert] generated for IP {ip} -> {cert_path}")
    return cert_path, key_path


if __name__ == "__main__":
    generate(force=True)
