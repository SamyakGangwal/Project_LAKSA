#!/usr/bin/env python3
"""Build one LAKSA car package and send it to the Jetson.

  python laksa_release.py build                 -> dist/car/laksa-car-<version>.tar.gz (+ .sha256)
  python laksa_release.py push [package]        -> copy to the car's ~/laksa/incoming
  python laksa_release.py push --now            -> also install it now and restart the car stack
  python laksa_release.py status                -> what the car is running

The package is made from the committed HEAD (``git archive``), so it is exactly
a commit: uncommitted changes are refused unless --allow-dirty.  On the car,
laksa-deploy.service installs the newest package at boot (see laksa_deploy.sh);
``push`` sets that service up the first time.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

PATHS = ("firmware/esp32-s3/jetson", "firmware/esp32-s3/extra_ros_packages/laksa_interfaces")
DEFAULT_HOST = "samyak@100.78.235.3"
SSH_OPTS = ["-o", "ConnectTimeout=20", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=15"]


def git(repo: Path, *args: str, binary: bool = False):
    out = subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
    return out.stdout if binary else out.stdout.decode().strip()


def build(repo: Path, out_dir: Path, allow_dirty: bool) -> Path:
    dirty = git(repo, "status", "--porcelain", "--", *PATHS)
    if dirty and not allow_dirty:
        sys.exit(f"uncommitted changes in the packaged paths (commit them or pass --allow-dirty):\n{dirty}")
    commit = git(repo, "rev-parse", "HEAD")
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M")
    name = f"laksa-car-{stamp}-{commit[:7]}"
    manifest = {
        "name": name, "commit": commit, "branch": git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "subject": git(repo, "log", "-1", "--format=%s"), "built_utc": stamp,
        "dirty_excluded": bool(dirty), "paths": list(PATHS),
    }
    archive = git(repo, "archive", "--format=tar", f"--prefix={name}/src/", "HEAD", *PATHS, binary=True)
    install = git(repo, "show", "HEAD:firmware/esp32-s3/jetson/release/install.sh", binary=True)

    out_dir.mkdir(parents=True, exist_ok=True)
    package = out_dir / f"{name}.tar.gz"
    with tarfile.open(package, "w:gz") as tar:
        def add_bytes(path: str, data: bytes, mode: int) -> None:
            info = tarfile.TarInfo(path)
            info.size, info.mode, info.mtime = len(data), mode, int(dt.datetime.now().timestamp())
            tar.addfile(info, io.BytesIO(data))
        root = tarfile.TarInfo(name)
        root.type, root.mode = tarfile.DIRTYPE, 0o755
        tar.addfile(root)
        add_bytes(f"{name}/install.sh", install, 0o755)
        add_bytes(f"{name}/MANIFEST.json", json.dumps(manifest, indent=2).encode() + b"\n", 0o644)
        with tarfile.open(fileobj=io.BytesIO(archive)) as source:
            for member in source.getmembers():
                if member.name.startswith(f"{name}/"):       # skips git's pax header
                    tar.addfile(member, source.extractfile(member) if member.isfile() else None)
    digest = hashlib.sha256(package.read_bytes()).hexdigest()
    # Bytes, not text: on Windows write_text would end the line with \r\n, which
    # sha256sum -c on the Jetson reads as part of the file name.
    package.with_name(package.name + ".sha256").write_bytes(f"{digest}  {package.name}\n".encode())
    print(f"{package}  ({package.stat().st_size / 1e6:.1f} MB, commit {commit[:7]})")
    return package


def ssh(host: str, command: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["ssh", *SSH_OPTS, host, command], check=check)


def push(package: Path, host: str, now: bool) -> None:
    name = package.name[: -len(".tar.gz")]
    checksum = package.with_name(package.name + ".sha256")
    if not checksum.exists():
        sys.exit(f"missing {checksum}")
    ssh(host, "mkdir -p ~/laksa/incoming")
    subprocess.run(["scp", *SSH_OPTS, str(package), str(checksum), f"{host}:laksa/incoming/"], check=True)
    print(f"copied {package.name} to {host}:~/laksa/incoming")
    # First time on this Jetson: install the deployer and its boot service.
    ssh(host, "test -x ~/laksa/bin/laksa_deploy.sh || ("
              f"t=$(mktemp -d) && tar -xzf ~/laksa/incoming/{package.name} -C $t "
              f"{name}/src/firmware/esp32-s3/jetson/release/laksa_deploy.sh "
              f"{name}/src/firmware/esp32-s3/jetson/systemd/laksa-deploy.service && "
              f"bash $t/{name}/src/firmware/esp32-s3/jetson/release/laksa_deploy.sh bootstrap $t/{name}; "
              "rc=$?; rm -rf $t; exit $rc)")
    if now:
        print("installing now (build + tests take a few minutes; the car stack restarts afterwards)")
        ssh(host, "bash ~/laksa/bin/laksa_deploy.sh boot && bash ~/laksa/bin/laksa_deploy.sh status "
                  "&& sudo -n systemctl restart laksa-car.service")
    else:
        print("it installs at the next boot (or run: push --now)")


def main() -> None:
    repo = Path(git(Path(__file__).resolve().parent, "rev-parse", "--show-toplevel"))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--out", type=Path, default=repo / "dist" / "car")
    b.add_argument("--allow-dirty", action="store_true")
    p = sub.add_parser("push")
    p.add_argument("package", nargs="?", type=Path, help="default: newest in dist/car")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--now", action="store_true", help="install now and restart the car stack")
    s = sub.add_parser("status")
    s.add_argument("--host", default=DEFAULT_HOST)
    args = parser.parse_args()

    if args.cmd == "build":
        build(repo, args.out, args.allow_dirty)
    elif args.cmd == "push":
        package = args.package or max((repo / "dist" / "car").glob("laksa-car-*.tar.gz"),
                                      key=lambda p: p.stat().st_mtime, default=None)
        if package is None:
            sys.exit("no package: run build first")
        push(package, args.host, args.now)
    else:
        ssh(args.host, "bash ~/laksa/bin/laksa_deploy.sh status", check=False)


if __name__ == "__main__":
    main()
