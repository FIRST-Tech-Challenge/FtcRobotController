# 0003: Tests and checks from day one, aimed at the coach

Status: Accepted. Test harness built. Style checks and architecture rules not built yet.
Date: 2026-09-26

## Context

The coach took over without a hand-off. Code will come across from Decode piece by piece,
some ported and some reworked, often with an AI coding assistant helping. The heavy build
and competition weeks are exactly when nobody will stop to add tests. So the checks have to
exist before the code does.

The checks are **for the coach**, to catch problems in ported and assistant-written code.
Students are not required to write tests.

## Decision

**Tests (built)**

- JUnit 5, Mockito, and jqwik, wired through `TeamCode/quality.gradle`: one `apply from:` line
  in `TeamCode/build.gradle`. Versions are pinned in that file.
- Run: `./gradlew :TeamCode:testDebugUnitTest`. Report:
  `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.
- Tests describe **current behavior**. If a test shows a likely bug, keep it, mark it
  `@Disabled("possible bug: ...")`, and decide the fix separately.
- Each piece brings its tests when it is ported. Tests written against Decode's behavior
  prove the port did not change it.

**CI (built)**

- `.github/workflows/build.yml` runs a `test` job next to the `build` job. Neither waits on the
  other. Reports upload on every run.
- **Not required to merge.** Branch protection is off while the team learns to build reliably.

**Style and architecture checks (planned, report only)**

- Formatting, Javadoc, and PMD rules for loop-time problems (hardware lookups, allocation, or
  string building inside `Process()`; `Thread.sleep`; `System.out`). They report counts and
  never fail the build.
- Architecture rules, also report only:
  - Season words (pollen, hive, flower, cluster, nectar) stay out of `objects/` and `opMode/`
    base classes.
  - Subsystems implement `Subsystem`, keep hardware fields private, and touch the hardware
    map only in the constructor or `Init()`.
  - No Decode leftovers (`VisionMode`, `DecodeAprilTag`, red/blue OpMode pairs) and no edits
    under `FtcRobotController/`.

## Consequences

- **jqwik trap:** jqwik silently skips `@Property` tests in a class that uses JUnit's
  `@DisplayName`, and the build stays green. Property tests go in their own class and use
  jqwik's `@Label`. Count results from the XML in `TeamCode/build/test-results/`, not the console.
- The `test` task always re-runs, because jqwik writes `TeamCode/.jqwik-database`
  (gitignored). That is expected.
- When CI is steady, turn on branch protection and make `Unit tests` a required check.
