#!/usr/bin/env python3
from pathlib import Path
import subprocess
import importlib.util
import sys
import os


def run(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def import_and_run_build():
    """Import build_release.py and call its main() function."""
    spec = importlib.util.spec_from_file_location("build_release", "build_release.py")
    if spec is None or spec.loader is None:
        sys.exit("Error: could not load build_release.py")

    build_release = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build_release)

    if hasattr(build_release, "main"):
        print("→ Running build_release.main()")
        build_release.main()
    else:
        sys.exit("Error: build_release.py has no main() function")


def main():
    repo_root = Path(__file__).parent.resolve()
    os.chdir(repo_root)
    # 1) Run build_release.main()
    import_and_run_build()

    # 2) Flash binary using ST-Link
    bin_path = Path("build/Release_Script/analog_tears.bin")
    if not bin_path.exists():
        sys.exit(f"Error: binary not found at {bin_path}")

    # Use st-flash to flash the binary
    # Format: st-flash write <file.bin> <address>
    # Address 0x08000000 is the standard STM32 flash start address
    run([
        "st-flash",
        "write",
        str(bin_path),
        "0x08000000",
    ])


if __name__ == "__main__":
    main()

