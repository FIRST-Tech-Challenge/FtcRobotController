# BioBuzz Zenith project

Open the repository folder containing `zenith.json` in Zenith. `autos/` is the authoritative source for routines, the command catalog, fields, waypoints and routine profiles. Edit these files through the editor; APK assets and generated Java are deployment outputs.

- `integration-example.auto.json`: cleared-area authoring example with a curve, markers, parallel actions, waits, a branch and mechanism commands. Coordinates and dynamics are unmeasured examples.
- `pollen-navigation-test.auto.json`: separate camera-guided collection experiment. Team deployment retains its sole **Test: Pollen Navigation (Pedro + Ivy)** entry with the isolated `POLLEN_TEST` profile.
- New routines: full robot profile, one generated Driver Station entry in **Generated**. The legacy red/blue Route Required wrappers are disabled. `red-audience` and `red-far` are DRAFT match routes with placeholder coordinates, not yet deployed; see [the match autonomous plan](../doc/MatchAutonomousPlan.md).

Use **UI edit/save → team deployment → APK build/install**:

```powershell
npm ci --ignore-scripts
npm run zenith:validate
npm run zenith:deploy -- --dry-run
npm run zenith:deploy
npm run zenith:check
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
.\scripts\check-zenith-apk.ps1
```

The native editor Deploy button does not replace team deployment, which preserves the pollen profile and removes stale managed outputs after source renames/deletions. Source names must match lowercase kebab-case filenames. Saving source alone does not update the Control Hub.

Read the [coder guide](../doc/ZenithAuthoringGuide.md), [audit against the plan and official sources](../doc/ZenithIntegrationAudit.md), and [verification record](../doc/ZenithVerification.md). Pedro tuning blocks START until verified. Camera/shooting calibration and maximum deployed envelope remain unverified; editor preview does not prove those properties.
