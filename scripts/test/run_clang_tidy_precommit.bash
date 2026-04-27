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

# Packages linted by this hook. Keep in sync with:
#   - the `files:` regex in .pre-commit-config.yaml
#   - the changed-files regex in
#     .github/workflows/build-and-test-agnocastlib-components-heaphook.yaml
PKGS=(agnocastlib agnocast_components)

# ---------------------------------------------------------------------------
# Filter the staged files passed by pre-commit to the same set CI lints.
# Done before the toolchain probes below — those spawn clang++-14 and add
# noticeable latency, so a no-op invocation should bail early.
# ---------------------------------------------------------------------------
pairs=()
for f in "$@"; do
  for pkg in "${PKGS[@]}"; do
    if [[ "$f" =~ ^src/$pkg/.*\.(cpp|hpp)$ ]] && [[ "$f" != *"/test/"* ]]; then
      pairs+=("$pkg|$f")
      break
    fi
  done
done

if [ ${#pairs[@]} -eq 0 ]; then
  exit 0
fi

# ---------------------------------------------------------------------------
# Toolchain checks.
# ---------------------------------------------------------------------------
CLANG_TIDY_BIN="clang-tidy-14"
RUN_CLANG_TIDY_BIN="run-clang-tidy-14"

if ! command -v "$CLANG_TIDY_BIN" >/dev/null 2>&1 || \
   ! command -v "$RUN_CLANG_TIDY_BIN" >/dev/null 2>&1; then
  echo "ERROR: $CLANG_TIDY_BIN / $RUN_CLANG_TIDY_BIN not found." >&2
  echo "Install with: sudo apt-get install -y clang-tidy-14" >&2
  exit 1
fi

# Sanity check: must be LLVM 14.x, matching CI.
tidy_version_out="$("$CLANG_TIDY_BIN" --version)"
TIDY_VERSION=""
if [[ "$tidy_version_out" =~ version[[:space:]]+([0-9]+) ]]; then
  TIDY_VERSION="${BASH_REMATCH[1]}"
fi
if [ "$TIDY_VERSION" != "14" ]; then
  echo "ERROR: $CLANG_TIDY_BIN reports major version '$TIDY_VERSION', expected 14." >&2
  exit 1
fi

# Sanity check: clang-14 must be able to find libstdc++ headers like <atomic>.
# On Ubuntu 22.04 with gcc-12 present but libstdc++-12-dev missing, clang-14
# selects the gcc-12 toolchain and fails to resolve standard C++ headers,
# producing a flood of clang-diagnostic-error noise that masks real findings.
# Probe with clang++-14 when available so the user sees a clear install hint
# instead of hunting through unrelated lint errors.
if command -v clang++-14 >/dev/null 2>&1; then
  if ! echo '#include <atomic>' | clang++-14 -x c++ -fsyntax-only - >/dev/null 2>&1; then
    echo "ERROR: clang++-14 cannot find libstdc++ headers (e.g. <atomic>)." >&2
    echo "       On Ubuntu 22.04 this usually means gcc-12 is installed but" >&2
    echo "       libstdc++-12-dev is missing — clang-14 picks the gcc-12 toolchain" >&2
    echo "       and fails to resolve standard C++ headers." >&2
    echo "       Install with: sudo apt-get install -y libstdc++-12-dev" >&2
    exit 1
  fi
fi

# ---------------------------------------------------------------------------
# Lint each package. clang-tidy needs a compilation database.
# colcon writes one per package under build/<pkg>/compile_commands.json when
# built with -DCMAKE_EXPORT_COMPILE_COMMANDS=1. With --merge-install some
# setups also expose build/compile_commands.json. If neither exists the hook
# warns and skips — CI runs clang-tidy on the PR, so a stale local build
# should not block the commit.
# ---------------------------------------------------------------------------
BUILD_HINT="Build with: colcon build --packages-up-to agnocastlib agnocast_components --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

# Partition the input files by membership in a compile DB. JSON-aware
# (whitespace/formatting independent), canonicalises both the DB entries
# and the input paths via realpath so that symlinked workspaces match.
# Outputs one line per input file:
#   "+ <relpath>" — present in DB, will be linted
#   "- <relpath>" — absent  from DB, will be reported as a gap
partition_files_against_db() {
  python3 - "$@" <<'PY'
import json, os, sys
db_path, repo, *files = sys.argv[1:]
with open(db_path) as fh:
    entries = json.load(fh)
in_db = set()
for e in entries:
    p = e.get("file")
    if not p:
        continue
    if not os.path.isabs(p):
        p = os.path.join(e.get("directory", ""), p)
    in_db.add(os.path.realpath(p))
for f in files:
    abs_p = os.path.realpath(os.path.join(repo, f))
    print(("+ " if abs_p in in_db else "- ") + f)
PY
}

JOBS="$(nproc)"
EXIT_CODE=0
for pkg in "${PKGS[@]}"; do
  files=()
  for pair in "${pairs[@]}"; do
    [ "${pair%%|*}" = "$pkg" ] && files+=("${pair#*|}")
  done
  [ ${#files[@]} -eq 0 ] && continue

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

  db="$P_DIR/compile_commands.json"

  # run-clang-tidy silently skips files absent from the compile DB, which
  # would let the hook report "Passed" without actually analysing them.
  # Partition staged files by DB membership and warn about the gap.
  matched_files=()
  missing_files=()
  while IFS= read -r line; do
    case "$line" in
      "+ "*) matched_files+=("${line#+ }") ;;
      "- "*) missing_files+=("${line#- }") ;;
    esac
  done < <(partition_files_against_db "$db" "$REPO_ROOT" "${files[@]}")

  if [ ${#missing_files[@]} -gt 0 ]; then
    if [ ${#matched_files[@]} -eq 0 ]; then
      echo "WARN: $db has no entries for the staged files in '$pkg' — skipping clang-tidy." >&2
    else
      echo "WARN: $db is missing entries for some staged files in '$pkg' — those files will not be linted locally." >&2
    fi
    for f in "${missing_files[@]}"; do
      echo "      missing: $f" >&2
    done
    echo "      $BUILD_HINT" >&2
    echo "      CI will still run clang-tidy on this PR." >&2
  fi

  [ ${#matched_files[@]} -eq 0 ] && continue

  # Pass -clang-tidy-binary explicitly so the wrapper does not fall back to
  # an unsuffixed `clang-tidy` (which on multi-version systems may resolve
  # to a different LLVM major version than the one we just validated).
  if ! "$RUN_CLANG_TIDY_BIN" -j "$JOBS" -p "$P_DIR" \
      -clang-tidy-binary "$CLANG_TIDY_BIN" "${matched_files[@]}"; then
    EXIT_CODE=1
  fi
done

exit "$EXIT_CODE"
