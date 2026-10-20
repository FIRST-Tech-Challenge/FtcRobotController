# Setting up a team laptop (Windows)

One laptop per subteam, both on the same branch. Do steps 1 to 4 once, at home or
anywhere with internet. Steps 5 to 7 are every practice.

| Laptop | Branch | Subteam |
|---|---|---|
| Drive laptop | `u/bryanculver-bot` | Drive and auton |
| Launcher laptop | `u/bryanculver-bot` | Launcher and aim assist |

Both subteams work on different files, so sharing a branch is fine. Pull (Update Project)
before you start, push when you finish, and the other laptop sees your work next pull.

## 1. Install (once, needs internet)

1. **Android Studio** from developer.android.com. Accept the defaults.
2. **Git for Windows** from git-scm.com. Accept the defaults. Android Studio needs it to see
   the branches.

## 2. Get the code (once, needs internet)

1. Android Studio > **File > New > Project from Version Control**.
2. URL: `https://github.com/FTC23623/BioBuzz.git`. Clone.
3. Bottom right of Android Studio shows the branch name. Click it > **Remote** > your
   laptop's branch from the table > **Checkout**.
4. **Check:** the branch name at the bottom right is your branch.

## 3. First build (once, needs internet)

1. **Build > Make Project** (the hammer). The first build downloads the libraries the
   robot code uses. It takes a few minutes.
2. **Check:** `BUILD SUCCESSFUL` at the bottom. After this, the laptop can build with no
   internet, which is what the robot's Wi-Fi is.

## 4. Find adb (once)

`adb` is the program that talks to the Control Hub. Android Studio installed it here:

```
C:\Users\<your name>\AppData\Local\Android\Sdk\platform-tools\adb.exe
```

Make it one click: Android Studio > **File > Settings > Tools > External Tools** > `+`.
Name `Connect to robot`, Program: the path above, Arguments: `connect 192.168.43.1`.
It then lives in the **Tools** menu.

## 5. Connect to the robot (every practice)

1. Robot on. Join the laptop to the **robot's Wi-Fi**, the same network the Driver Station
   joins, with the same password. The laptop has no internet while it is there.
2. **Tools > Connect to robot**. It should say `connected to 192.168.43.1:5555`.
   "already connected" is fine too.
3. **Check:** the device dropdown at the top of Android Studio says `REV Robotics Control
   Hub`.

If it says `failed to connect`, the laptop is not on the robot's Wi-Fi. Windows likes to
hop back to the school network: join the robot's again. If the hub rebooted or the laptop
slept, just run Connect to robot again.

Android Studio's own "Pair Devices Using Wi-Fi" never works with the hub. It needs Android
11; the hub runs Android 7. Connect to robot is the way.

## 6. Deploy and drive (every practice)

1. Press **Run** (the green triangle). About a minute. The Driver Station's OpMode list
   updates by itself.
2. **The robot holds one program at a time.** When the other laptop deploys, your OpModes
   are gone from the list until you press Run again. Take turns: say "deploying" out loud.
3. Safe Mode key in whenever the robot is on the floor. Wheels off the ground for any test
   that says so.

## 7. Save your work (every practice, before you pack up)

1. **Build** first. Never save a red build.
2. **Git > Commit** (Ctrl+K). Tick your files. Write what changed and why, in plain words:
   `Launcher: start at 3000 RPM for the new wheel`. Commit.
3. **Git > Push** (Ctrl+Shift+K). Needs internet: do it when you are back on a network with
   internet, or on a phone hotspot.
4. Next practice, before anything else: **Git > Update Project** (Ctrl+T) to pull what the
   coach merged.

## Rules

- **Stay on `u/bryanculver-bot`.** Do not make new branches from a laptop; the coach does that.
- **Only the coach merges** into `u/bryanculver-bot` and `master`.
- **Every number has a name** in `BotProperties.java`. A test OpMode that needs a different
  number sets it at the top of its `runOpMode()`, like `Test: Launcher` does.
- Stuck? Read `docs/how-the-code-fits-together.html` (open it in a browser), then ask.
