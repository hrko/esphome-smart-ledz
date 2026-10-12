# C++ static analysis

Run `mise install` followed by `mise run cpp-check`. Use mise 2026.10.3 or newer
for the version 3 lockfile. CI uses Linux; native analysis requires the host C++
standard library headers (g++ on Linux, Visual Studio C++ tools on Windows).

The task checks every production `.cpp` file and a translation unit including
all public `.h` files, with Clang `-Wall -Wextra -Werror` and clang-tidy.
Missing compilation entries, parsing errors, and enabled diagnostics fail the
check. SDK and third-party headers are system includes; owned public headers
remain eligible for diagnostics. No automatic fixes run in CI.

Clang and clang-tidy are pinned together in mise.toml and mise.lock. Rules focus
on the Clang Static Analyzer, selected bugprone checks, and unnecessary copies.
Suppressions should be narrow and explain why the diagnostic is inapplicable.

ESPHome and ESP-IDF analysis first builds the ESP32-S3 validation firmware and
uses its compile_commands.json, definitions, language mode, and SDK headers.
As in ESPHome's script/clang-tidy, Xtensa is approximated with 32-bit x86 and
Xtensa architecture macros. This is not full Xtensa semantic verification;
the actual GCC firmware build remains required. The standalone protocol
library uses native C++17 settings and does not require ESPHome.

The mesh validation fixture loads this checkout via a local file URI, so a PR
checks its own source rather than a published library revision. The standalone
library workflows do not depend on a checkout of the hub repository.

Static analysis cannot establish that BLE callbacks will arrive or prove all
clock rollover and timeout behavior. Keep the existing regression tests and
real-device validation. Sanitizers, fuzzing, Cppcheck, and formatting enforcement
are outside this initial static-analysis change.

References:
- https://github.com/esphome/esphome/blob/2026.9.1/script/clang-tidy
- https://github.com/hrko/flexispot-esphome/blob/3088cfedfabae78accf0ad732bdea66dc4b84f05/tools/check_cpp.py
- https://clang.llvm.org/extra/clang-tidy/
