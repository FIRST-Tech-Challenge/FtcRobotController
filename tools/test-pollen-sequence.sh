#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
test_build_dir="$(mktemp -d)"
trap 'rm -rf "$test_build_dir"' EXIT
src="$repo_dir/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen"
javac --release 8 -d "$test_build_dir" "$src/PollenSequence.java" "$src/PollenConfig.java" "$src/PollenParkTuning.java" "$src/PollenShootTuning.java" \
  "$repo_dir/tools/pollen-tests/PollenSequenceTest.java" \
  "$repo_dir/tools/pollen-tests/PollenParkTuningTest.java" \
  "$repo_dir/tools/pollen-tests/PollenShootFirstTest.java"
java -cp "$test_build_dir" PollenSequenceTest
java -cp "$test_build_dir" PollenParkTuningTest
java -cp "$test_build_dir" PollenShootFirstTest
