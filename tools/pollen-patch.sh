#!/usr/bin/env bash
# Run from the team checkout. Compatible with macOS Bash 3.2.
# Usage: bash /path/pollen-patch.sh apply /path/change.patch
#        bash /path/pollen-patch.sh verify
# FTC_JAVA_HOME overrides Android Studio's bundled JDK.
# FTC_OFFLINE=1 adds Gradle --offline after dependencies have been cached.
set -euo pipefail

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
usage() {
  printf 'Usage: bash %s apply PATCH_FILE | verify\n' "$0"
  printf 'Run inside FtcBagelBits/FtcRobotController on pollen-auto-prep.\n'
}

action="${1:-}"
case "$action" in
  apply) [ "$#" -eq 2 ] || { usage; exit 2; } ;;
  verify) [ "$#" -eq 1 ] || { usage; exit 2; } ;;
  *) usage; exit 2 ;;
esac

patch_file=''
if [ "$action" = apply ]; then
  [ -f "$2" ] || die "Patch file not found: $2"
  patch_dir="$(cd "$(dirname "$2")" && pwd)"
  patch_file="$patch_dir/$(basename "$2")"
fi

repo_dir="$(git rev-parse --show-toplevel 2>/dev/null)" || die 'Run this from inside your team checkout.'
cd "$repo_dir"
[ -f tools/test-pollen-sequence.sh ] && [ -f TeamCode/build.gradle ] || die 'This is not the expected pollen project.'
origin="$(git remote get-url origin 2>/dev/null)" || die 'Missing origin remote.'
case "$origin" in
  https://github.com/FtcBagelBits/FtcRobotController|https://github.com/FtcBagelBits/FtcRobotController.git|git@*:FtcBagelBits/FtcRobotController.git|ssh://git@*/FtcBagelBits/FtcRobotController.git) ;;
  *) die 'Origin is not FtcBagelBits/FtcRobotController. Check git remote -v.' ;;
esac
branch="$(git symbolic-ref --quiet --short HEAD)" || die 'Detached HEAD; switch to pollen-auto-prep first.'
[ "$branch" = pollen-auto-prep ] || die "Expected pollen-auto-prep; currently $branch. Switch branches yourself first."

ftc_jdk="${FTC_JAVA_HOME:-/Applications/Android Studio.app/Contents/jbr/Contents/Home}"
[ -x "$ftc_jdk/bin/java" ] && [ -x "$ftc_jdk/bin/javac" ] || die 'A full JDK is required. Set FTC_JAVA_HOME to its Contents/Home directory.'
export JAVA_HOME="$ftc_jdk"
export PATH="$ftc_jdk/bin:$PATH"
printf 'Repository: %s\nBranch: %s\nJDK: %s\n' "$repo_dir" "$branch" "$ftc_jdk"
java -version

if [ "$action" = apply ]; then
  if git apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    printf 'Patch is already present; proceeding to verification without applying it again.\n'
  else
    if [ -n "$(git status --porcelain --untracked-files=normal)" ]; then
      git status --short
      die 'Working tree has changes. Review and commit your intended work first; keep downloaded patch files outside the repo. Nothing was stashed or discarded.'
    fi
    git apply --check "$patch_file" || die 'Patch does not apply cleanly. Nothing was applied. Share this error; do not force it.'
    backup_branch="backup/pollen-before-$(date +%Y%m%d-%H%M%S)-$$"
    git branch "$backup_branch" HEAD
    printf 'Saved pre-patch commit as %s\n' "$backup_branch"
    git apply "$patch_file"
    printf 'Patch applied. Changes remain uncommitted for review.\n'
  fi
fi

log_file="$(mktemp "${TMPDIR:-/tmp}/pollen-verify.XXXXXX")"
verify_project() {
  git diff --check HEAD || return $?
  bash tools/test-pollen-sequence.sh || return $?
  if [ "${FTC_OFFLINE:-0}" = 1 ]; then
    bash ./gradlew "-Dorg.gradle.java.home=$ftc_jdk" --offline :TeamCode:assembleDebug || return $?
  else
    bash ./gradlew "-Dorg.gradle.java.home=$ftc_jdk" :TeamCode:assembleDebug || return $?
  fi
}
if verify_project 2>&1 | tee "$log_file"; then
  printf '\nVerification passed. Log: %s\n' "$log_file"
  printf 'Review: git diff --stat; git diff; git status --short\n'
  printf 'APK: TeamCode/build/outputs/apk/debug/ (use the freshly built APK).\n'
  printf 'No app was installed and no commit or push was performed.\n'
else
  printf '\nVerification failed. Log: %s\n' "$log_file" >&2
  printf 'Any applied changes remain for inspection. Do not install this build until fixed.\n' >&2
  exit 1
fi
