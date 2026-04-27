#!/bin/bash
# Pre-commit hook: run clang-tidy with the SAME version as CI.
#
# CI invokes `run-clang-tidy-14` (LLVM 14, Ubuntu 22.04 / Humble) — see
# .github/workflows/build-and-test-agnocastlib-components-heaphook.yaml.
# This script enforces the same major version locally so issues caught
# in CI are also caught at commit time, and vice versa.

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

CLANG_TIDY_BIN="clang-tidy-14"
RUN_CLANG_TIDY_BIN="run-clang-tidy-14"

if ! command -v "$CLANG_TIDY_BIN" >/dev/null 2>&1 || \
   ! command -v "$RUN_CLANG_TIDY_BIN" >/dev/null 2>&1; then
  echo "ERROR: $CLANG_TIDY_BIN / $RUN_CLANG_TIDY_BIN not found." >&2
  echo "Install with: sudo apt-get install -y clang-tidy-14" >&2
  exit 1
fi

# Sanity check: must be LLVM 14.x, matching CI.
TIDY_VERSION="$("$CLANG_TIDY_BIN" --version | grep -oE 'version [0-9]+' | head -n1 | awk '{print $2}')"
if [ "$TIDY_VERSION" != "14" ]; then
  echo "ERROR: $CLANG_TIDY_BIN reports major version '$TIDY_VERSION', expected 14." >&2
  exit 1
fi

# Filter the staged files passed by pre-commit to the same set CI lints:
#   src/(agnocastlib|agnocast_components)/**/*.{cpp,hpp}, excluding /test/.
agnocastlib_files=()
agnocast_components_files=()
for f in "$@"; do
  if [[ "$f" =~ ^src/(agnocastlib|agnocast_components)/.*\.(cpp|hpp)$ ]] && \
     [[ "$f" != *"/test/"* ]]; then
    pkg="${f#src/}"; pkg="${pkg%%/*}"
    case "$pkg" in
      agnocastlib)         agnocastlib_files+=("$f") ;;
      agnocast_components) agnocast_components_files+=("$f") ;;
    esac
  fi
done

if [ ${#agnocastlib_files[@]} -eq 0 ] && [ ${#agnocast_components_files[@]} -eq 0 ]; then
  exit 0
fi

# clang-tidy needs a compilation database. colcon writes one per package
# under build/<pkg>/compile_commands.json when built with
# -DCMAKE_EXPORT_COMPILE_COMMANDS=1. With --merge-install some setups also
# expose build/compile_commands.json. If neither exists the hook prints a
# warning and skips — CI runs clang-tidy on the PR, so a stale local build
# should not block the commit.
BUILD_HINT="Build with: colcon build --packages-up-to agnocastlib agnocast_components --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

JOBS="$(nproc)"
EXIT_CODE=0
for pkg in agnocastlib agnocast_components; do
  declare -n pkg_files="${pkg}_files"
  [ ${#pkg_files[@]} -eq 0 ] && continue

  if [ -f "build/$pkg/compile_commands.json" ]; then
    P_DIR="build/$pkg"
  elif [ -f "build/compile_commands.json" ]; then
    P_DIR="build"
  else
    echo "WARN: no compile_commands.json for package '$pkg' — skipping clang-tidy." >&2
    echo "      Looked for build/$pkg/compile_commands.json and build/compile_commands.json." >&2
    echo "      $BUILD_HINT" >&2
    echo "      CI will still run clang-tidy on this PR." >&2
    continue
  fi

  if ! "$RUN_CLANG_TIDY_BIN" -j "$JOBS" -p "$P_DIR" "${pkg_files[@]}"; then
    EXIT_CODE=1
  fi
done

exit "$EXIT_CODE"
