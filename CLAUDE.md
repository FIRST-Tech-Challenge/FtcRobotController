# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A team fork of the FIRST Tech Challenge SDK (`FtcRobotController` v11.2, DECODE 2025-2026 season). It builds an Android APK that runs on the Robot Controller phone / Control Hub. Team-authored robot code lives in `TeamCode`; everything under `FtcRobotController/` is vendor SDK code and samples, left alone so upstream SDK updates merge cleanly.

Two remotes, and the distinction matters:

- `origin` → `ngi-collective/FtcRobotController2026-2027` — the team fork. All team work goes here.
- `upstream` → `FIRST-Tech-Challenge/FtcRobotController` — the official SDK, pull-only.

`.github/CONTRIBUTING.md` is upstream's, and its main point applies: team code is never meant to go back to `upstream`. Never push or open a PR against it.

## Commands

**Every Gradle invocation must run under mise** — it supplies both the JDK and the Android SDK, and the build fails without it (see Toolchain). Prefer the tasks in `mise.toml`, which run inside that environment already:

```bash
mise run check       # compile TeamCode - fastest correctness check while editing OpModes
mise run test        # unit tests (currently NO-SOURCE, see below)
mise run build       # debug APK
mise run install     # build + adb install to a connected Robot Controller device
mise run lint
mise run clean
mise run setup-sdk         # (re)install the SDK packages the build needs
mise run verify-toolchain  # assert CLI and IDE toolchains can both still build (CI gate)

mise tasks           # list the above with descriptions
```

Gradle flags pass straight through, with or without a `--` separator: `mise run build --info`, `mise run check -- --dry-run`.

Every task targets `:TeamCode:` specifically, since `TeamCode` is the only application module — `FtcRobotController` applies `com.android.library`. Raw Gradle still works if a task doesn't cover the case, but needs the environment: `mise exec -- ./gradlew <task>`.

There is no unit test suite. `mise run test` invokes `:TeamCode:testDebugUnitTest`, which reports `NO-SOURCE` and passes — it is wired into CI so the gate starts enforcing the moment a real test lands, but today it proves nothing. Real verification happens on the robot via Driver Station telemetry. Treat "does it compile" as the only meaningful automated gate, and say so explicitly when a change has only been compile-checked.

## CI

`.github/workflows/ci.yml` runs on pushes to `master`, on PRs, and on demand. It installs the toolchain with `jdx/mise-action`, so CI and local builds run the same `mise.toml` pins and the same tasks: `verify-toolchain` → `test` → `lint` → `build`. Adding a task to `mise.toml` is what changes CI behavior; the workflow itself carries no build logic.

The Android SDK packages that `setup-sdk` installs are cached separately from the mise tool install, keyed on `mise.toml` + `build.common.gradle`. Every run uploads the lint report, and a successful run uploads `TeamCode-debug.apk` — a team member can install from a CI run without a local Android toolchain.

`.github/dependabot.yml` covers GitHub Actions only. The `org.firstinspires.ftc.*` artifacts are deliberately excluded: they are locked to the season's SDK release and must move together as an upstream merge, never one bot PR at a time.

Toolchain: Gradle 8.9, AGP 8.7.0, `compileSdk 30` / `minSdk 24` / `targetSdk 28`, build-tools 34.0.0, Java 8 source and target compatibility.

**Run Gradle under JDK 21.** `mise.toml` pins `java = "temurin-21"` for exactly this reason. The system JDK is 25, and Gradle 8.9 cannot read class file major version 69 — any build script it has to compile fresh dies with `Unsupported class file major version 69`. Cached scripts mask this, so a build can appear healthy right up until `clean`. AGP 8.7.0 independently requires JDK 17+, making 21 the LTS that satisfies both. If mise is not active in the shell, prefix commands with `mise exec --`.

**mise owns the Android SDK too.** `mise.toml` pins `android-sdk = "22.0"` and exports `ANDROID_HOME` at that install. `sdk.dir` is deliberately left unset in `local.properties`, because it takes precedence over `ANDROID_HOME` and would silently reintroduce a second SDK. Outside a mise environment the build now fails with `SDK location not found` — that error means mise is not active, not that anything is misconfigured.

The mise `android-sdk` plugin ships **cmdline-tools only**. The platform and build-tools are installed underneath it by `mise run setup-sdk`, which accepts the SDK licenses and installs `platforms;android-30`, `build-tools;34.0.0`, and `platform-tools`. Those live inside the versioned install directory, so bumping the pinned `android-sdk` version yields an empty SDK until `setup-sdk` is rerun — that is why the version is pinned rather than `latest`.

Android Studio may rewrite `local.properties` and re-add `sdk.dir` pointing at its own SDK. If the CLI and the IDE start disagreeing, that is the cause; delete the line again or point it at `mise where android-sdk`.

**The IDE is not on the mise toolchain, because wiring it up is fiddly — not because the split is wanted.** Android Studio takes its Gradle JVM from `.gradle/config.properties` (`java.home` → the bundled JBR) and its SDK from its own IDE setting, and it inherits no shell environment when launched from the Dock. Pointing both at mise means hand-editing IDE settings that reference mise's versioned install paths, which then break on any `mise.toml` version bump. Until that is worth doing, the two toolchains drift independently, and `mise run verify-toolchain` is the guard: it asserts each side can still build the project — JDK within 17-21, and an SDK containing `android-30` plus build-tools 34.0.0 — rather than asserting they match. The required platform is derived from `compileSdkVersion` in `build.common.gradle`, so it follows an SDK bump automatically. Every IDE-side input (`.gradle/`, `local.properties`, `.idea/`) is gitignored, so in CI those two checks report `skip` and only the mise side is enforced.

`ndkVersion '21.3.6528147'` is declared in `build.common.gradle`, but no NDK is installed and the build succeeds anyway — there are no native sources, so `mergeDebugNativeLibs` is `NO-SOURCE`. Don't chase an NDK install to fix an unrelated build failure.

## Module layout and build wiring

- `settings.gradle` includes exactly two modules: `:FtcRobotController` and `:TeamCode`.
- `build.common.gradle` holds all shared Android config and is explicitly upstream-owned — do not edit it. Module customization goes in `TeamCode/build.gradle`.
- `build.dependencies.gradle` pins every `org.firstinspires.ftc:*` artifact to a single SDK version (currently `11.2.0`). Bumping the SDK means bumping all of them together.
- `build.common.gradle` scrapes `versionCode` / `versionName` out of `FtcRobotController/src/main/AndroidManifest.xml` with a regex at configure time, so that manifest is the single source of app version truth.
- Only `mavenCentral()` and `google()` are declared. Third-party FTC libraries (Pedro Pathing, Panels, FTCLib, Road Runner) each need their own maven repo added before their coordinates will resolve.
- Debug and release both sign with `libs/ftc.debug.keystore` unless `APK_SIGNING_STORE_FILE` and friends are set in the environment.

## OpMode model

OpModes are discovered by annotation (`@TeleOp` / `@Autonomous`, optionally `@Disabled`), not by registration. `FtcRobotController/.../FtcOpModeRegister.java` exists only as a retired manual-registration hook and should stay empty. A new OpMode is just a new annotated class under `org.firstinspires.ftc.teamcode`.

Two OpMode styles appear in the samples: `LinearOpMode` (imperative `runOpMode()` with `waitForStart()` and an `opModeIsActive()` loop) and `OpMode` (callbacks `init` / `init_loop` / `start` / `loop`).

Hardware is resolved by string name against the configuration file stored on the Robot Controller device (`hardwareMap.get(DcMotorEx.class, "FL")`). A name mismatch is a runtime crash on init, not a compile error, so hardware names must be kept consistent across every OpMode and constants class in the module.

## TeamCode is currently empty

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` contains only `readme.md` — no OpModes. `compileDebugJavaWithJavac` reports `NO-SOURCE`. The repo is effectively unmodified upstream SDK plus this file.

`master` was reset to `upstream/master` (`4ed7c46`), which dropped the team's only commit, `2ef2666` "Added pedropathing and imu". That commit is still reachable through the reflog (`HEAD@{1}`) and holds:

- `iamyou.java` — field-centric mecanum TeleOp using motors `FL`/`FR`/`BL`/`BR` (left side `REVERSE`) and an IMU named `imu` mounted logo-UP / USB-FORWARD.
- `pedroPathing/Constants.java`, `pedroPathing/Tuning.java` — Pedro Pathing configuration and its tuner menu.

Before restoring any of it: those Pedro Pathing sources were committed **without** the corresponding gradle wiring. They import `com.pedropathing.*` and `com.bylazar.*`, which no repository or dependency in this project provides, so bringing them back as-is reintroduces ~100 `cannot find symbol` errors. Add the Pedro Pathing and Panels maven repos plus dependencies first.

## Conventions

Sample naming in `FtcRobotController/.../external/samples/` follows `Basic` / `Sensor` / `Robot` / `Concept` prefixes (plus some `Utility` classes); the scheme is documented in `sample_conventions.md` alongside them. Copy a sample into `TeamCode` rather than editing it in place.
