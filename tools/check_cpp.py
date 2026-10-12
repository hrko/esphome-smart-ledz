"""Check every production translation unit; SDK analysis follows ESPHome's Xtensa adapter."""

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(tool, args):
    executable = shutil.which(tool)
    if executable is None:
        raise RuntimeError(f"Missing {tool}; run mise install")
    with tempfile.TemporaryDirectory() as directory:
        response = Path(directory) / "arguments.rsp"
        response.write_text(
            subprocess.list2cmdline(args) if os.name == "nt" else shlex.join(args),
            encoding="utf-8",
        )
        subprocess.run([executable, f"@{response}"], cwd=ROOT, check=True)


def sdk_options(entry):
    from esphome.build_helpers.idedata import get_toolchain_includes, parse_entry

    compiler, defines, includes, groups = parse_entry(entry)
    if not Path(compiler).name.startswith("xtensa-"):
        raise ValueError(f"Expected Xtensa compiler, got {compiler}")
    flags = [flag for group in groups for flag in group.splitlines()]
    options = [
        "--target=i386-unknown-linux-gnu",
        "-U__i386__",
        "-U__x86_64__",
        "-D__XTENSA__",
        "-D__XTENSA_EL__",
        "-D_LIBC",
        "-nostdinc",
        "-nostdinc++",
        "-DCLANG_TIDY",
        "-D_GLIBCXX_HAVE_TLS",
    ]
    options += [
        f
        for f in flags
        if f.startswith("-std=") or f in ("-fno-exceptions", "-fno-rtti")
    ]
    options += [f"-D{define}" for define in defines]
    for include in includes + get_toolchain_includes(compiler):
        options += ["-isystem", include]
    return options


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path)
    args = parser.parse_args()
    component = (ROOT / "components").is_dir()
    base = ROOT / ("components" if component else "src")
    sources = sorted(base.rglob("*.cpp"))
    if not sources:
        raise ValueError("No production sources found")
    entries = json.loads(args.database.read_text()) if args.database else None
    if entries is None and (component or (ROOT / "include/esp_telink_mesh").exists()):
        raise ValueError(
            "ESPHome/ESP-IDF sources require --database from a real firmware build"
        )
    clang = shutil.which("clang")
    if clang is None:
        raise RuntimeError("Missing clang; run mise install")
    resource = subprocess.check_output(
        [clang, "-print-resource-dir"], text=True
    ).strip()
    for source in sources:
        if entries is not None:
            suffix = (
                "/esphome/" + source.relative_to(ROOT).as_posix()
                if component
                else "/src/" + source.name
            )
            matches = [
                e for e in entries if e["file"].replace("\\", "/").endswith(suffix)
            ]
            if len(matches) != 1:
                raise ValueError(
                    f"Expected exactly one build entry for {source}: {len(matches)}"
                )
            options = sdk_options(matches[0])
        else:
            options = ["-std=c++17", f"-resource-dir={resource}"]
            if sys.platform.startswith("linux"):
                # Conda Clang's sysroot does not contain libstdc++ headers.
                probe = subprocess.run(
                    ["g++", "-E", "-x", "c++", "-", "-v"],
                    input="",
                    text=True,
                    capture_output=True,
                    check=True,
                )
                search = probe.stderr.split("#include <...> search starts here:")[
                    1
                ].split("End of search list.")[0]
                for include in search.splitlines():
                    if include.strip():
                        options += ["-isystem", include.strip()]
        options = ["-I", str(ROOT / "include"), "-I", str(ROOT)] + options
        print(f"Checking {source.relative_to(ROOT)}", flush=True)
        run(
            "clang",
            [str(source), "-fsyntax-only", "-Wall", "-Wextra", "-Werror", *options],
        )
        run("clang-tidy", [str(source), "--", *options])
    # Parse all public headers, including ones not included by any .cpp file.
    headers = sorted((ROOT / ("components" if component else "include")).rglob("*.h"))
    if not headers:
        raise ValueError("No public headers found")
    with tempfile.TemporaryDirectory() as directory:
        header_unit = Path(directory) / "headers.cpp"
        header_unit.write_text(
            "".join(f'#include "{h.as_posix()}"\n' for h in headers), encoding="utf-8"
        )
        run(
            "clang",
            [
                str(header_unit),
                "-fsyntax-only",
                "-Wall",
                "-Wextra",
                "-Werror",
                *options,
            ],
        )
        run(
            "clang-tidy",
            [str(header_unit), f"--config-file={ROOT / '.clang-tidy'}", "--", *options],
        )
    print(
        f"Checked {len(sources)} production sources and {len(headers)} public headers"
    )


if __name__ == "__main__":
    main()
