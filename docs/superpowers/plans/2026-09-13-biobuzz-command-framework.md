# BIOBUZZ Command Framework Implementation Plan

> Historical document: FTC Knowledge Bank was removed from this project on 2026-10-28. FTCKB integration steps, rules, and check commands below are retained as history and no longer apply.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the pinned FTC Knowledge Bank and build a small, team-owned command framework plus a safe, minimal mecanum TeleOp under `TeamCode`.

**Architecture:** A pure-Java command core owns scheduling and subsystem requirements without depending on FTCLib. A drive feature composes a pure mecanum mixer, an FTC SDK-backed subsystem, and a supplier-driven default command; a `LinearOpMode` owns lifecycle and guaranteed shutdown.

**Tech Stack:** Java 8 source compatibility, FTC SDK 12.0, Android Gradle Plugin 8.13.2, Gradle 9.1, JUnit 4.13.2, Python 3.14 with jsonschema 4.26, JDK 21, FTC Knowledge Bank project integration v2.

## Global Constraints

- Pin FTC Knowledge Bank commit `105c8e47ddee734365af50dd2305caa13646ab21`; never track `main` or run `submodule update --remote`.
- Configure team `20827`, season `2026-2027`, and explicit profile `command-based`.
- Complete FTCKB dry-run, installation, and `resolve` before modifying `TeamCode`.
- Do not add FTCLib as a runtime dependency and do not recreate the full FTCLib/WPILib API.
- Keep hardware names, directions, dimensions, gearing, PID values, servo endpoints, odometry offsets, and power limits centralized and explicitly marked unverified until checked on the current robot.
- Do not commit `local.properties`, personal SDK/JDK paths, API keys, hooks, Gradle caches, or generated build output.
- A successful unit test or Android build is software evidence only; it is not deployment or real-robot validation.

## File map

- `.gitmodules`: records the pinned FTCKB submodule location.
- `.ftckb/project.yaml`: machine-readable team, season, profile, source pin, and managed-file hashes.
- `.agents/skills/ftc-knowledge-bank/SKILL.md`: generated project runtime skill.
- `AGENTS.md`: generated FTCKB managed block plus any future team-owned instructions outside it.
- `tools/FTC-Knowledge-Bank`: fixed-revision knowledge and verification submodule.
- `TeamCode/build.gradle`: adds only the JUnit dependency needed by local unit tests.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/Subsystem.java`: subsystem lifecycle contract.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/Command.java`: command lifecycle and requirement contract.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/CommandBase.java`: immutable requirement-set helper.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/CommandScheduler.java`: instance-owned scheduler, conflicts, defaults, cancellation, and shutdown.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/WheelPowers.java`: immutable four-wheel power value.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/MecanumMixer.java`: pure, normalized robot-centric mecanum mixing.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/DriveSubsystem.java`: sole owner of drivetrain motor output.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/TeleopDriveCommand.java`: live gamepad input, deadband, scaling, and drivetrain command.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotConfig.java`: unverified hardware-map names, directions, and drive settings.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotHardware.java`: typed SDK hardware lookup and safe motor initialization.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java`: scheduler and OpMode lifecycle composition root.
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/framework/command/CommandSchedulerTest.java`: command lifecycle, conflict, default, cancellation, and shutdown tests.
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/drive/MecanumMixerTest.java`: drivetrain math tests.
- `TeamCode/README.md`: teammate setup, configuration, validation status, and safe first-run checklist.

---

### Task 1: Integrate and resolve FTC Knowledge Bank

**Files:**
- Create: `.gitmodules`
- Create: `.ftckb/project.yaml`
- Create: `.agents/skills/ftc-knowledge-bank/SKILL.md`
- Create or modify: `AGENTS.md`
- Create: `tools/FTC-Knowledge-Bank` gitlink

**Interfaces:**
- Consumes: reviewed upstream source commit `105c8e47ddee734365af50dd2305caa13646ab21` through its `.agents/skills/ftckb-integrate/scripts/integrate.py` entry point.
- Produces: `project.py resolve --project .`, `project.py check --project .`, and `verify.py --project .` entry points pinned in the target repository.

- [ ] **Step 1: Record the clean target baseline**

Run:

```bash
git status --short --branch
git diff --cached --name-status
git submodule status
```

Expected: branch `master` is ahead only by the approved design/plan commits; no unrelated tracked or untracked changes. Stop and preserve any unexpected user change.

- [ ] **Step 2: Run the installer preview with the reviewed source**

Fetch the exact source into a temporary location and verify the checkout before invoking it:

```bash
test ! -e /private/tmp/ftckb-source-105c8e47
git clone --no-checkout https://github.com/lucasnotfound59/FTC-Knowledge-Bank.git /private/tmp/ftckb-source-105c8e47
git -C /private/tmp/ftckb-source-105c8e47 checkout --detach 105c8e47ddee734365af50dd2305caa13646ab21
git -C /private/tmp/ftckb-source-105c8e47 rev-parse HEAD
```

Expected: `rev-parse` prints exactly `105c8e47ddee734365af50dd2305caa13646ab21`. If the fixed temporary path already exists, inspect its remote and HEAD rather than deleting or trusting it.

Run:

```bash
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
python3 /private/tmp/ftckb-source-105c8e47/.agents/skills/ftckb-integrate/scripts/integrate.py \
  --project '/Users/xinlu/Documents/GitHub/FTC20287-BioBuzz' \
  --team 20827 \
  --season 2026-2027 \
  --ref 105c8e47ddee734365af50dd2305caa13646ab21 \
  --profile command-based \
  --dry-run
```

Expected: exit 0; the JSON plan identifies `.ftckb/project.yaml`, `.agents/skills/ftc-knowledge-bank/SKILL.md`, the managed `AGENTS.md` block, `.gitmodules`, and `tools/FTC-Knowledge-Bank`; it does not edit Android Gradle files.

- [ ] **Step 3: Execute the exact reviewed installation**

Run the same command without `--dry-run`:

```bash
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
python3 /private/tmp/ftckb-source-105c8e47/.agents/skills/ftckb-integrate/scripts/integrate.py \
  --project '/Users/xinlu/Documents/GitHub/FTC20287-BioBuzz' \
  --team 20827 \
  --season 2026-2027 \
  --ref 105c8e47ddee734365af50dd2305caa13646ab21 \
  --profile command-based
```

Expected: `installationOk=true`; the source `commit` and submodule HEAD are the exact 40-character SHA; no commit or push occurs automatically. If `projectCheck.ok=false`, record its hard-rule output rather than claiming full compliance.

- [ ] **Step 4: Resolve active development rules before coding**

Run:

```bash
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
python3 tools/FTC-Knowledge-Bank/.agents/skills/ftckb-integrate/scripts/project.py resolve --project .
```

Expected: exit 0 kernel JSON for team `20827`, season `2026-2027`, and normalized profile `command-based`. Save the active rule IDs in the execution notes and apply them to every following task.

- [ ] **Step 5: Verify integration state and staging scope**

Run:

```bash
git submodule status -- tools/FTC-Knowledge-Bank
git diff --check
git status --short
```

Expected: the submodule line begins with `105c8e47`; `.gitmodules` and the gitlink may already be staged by `git submodule add`; generated project files are visible and no unrelated paths changed.

- [ ] **Step 6: Commit only the project integration**

```bash
git add -- .gitmodules .ftckb/project.yaml .agents/skills/ftc-knowledge-bank/SKILL.md AGENTS.md tools/FTC-Knowledge-Bank
git diff --cached --check
git diff --cached --stat
git commit -m "chore: integrate FTC Knowledge Bank"
```

Expected: one commit containing only FTCKB integration artifacts.

---

### Task 2: Build the pure-Java command core with tests

**Files:**
- Modify: `TeamCode/build.gradle`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/Subsystem.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/Command.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/CommandBase.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework/command/CommandScheduler.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/framework/command/CommandSchedulerTest.java`

**Interfaces:**
- Consumes: Java 8 collections and lifecycle callbacks only.
- Produces: `Subsystem.periodic()`, `Subsystem.stop()`, `Command.initialize()`, `Command.execute()`, `Command.end(boolean)`, `Command.isFinished()`, `Command.getRequirements()`, `CommandBase.addRequirements(Subsystem...)`, and scheduler registration/default/schedule/run/cancel/shutdown methods.

- [ ] **Step 1: Add the local unit-test dependency**

Add inside the existing `dependencies` block in `TeamCode/build.gradle`:

```groovy
testImplementation 'junit:junit:4.13.2'
```

- [ ] **Step 2: Write failing scheduler contract tests**

Create `CommandSchedulerTest.java` with test doubles and these assertions:

```java
package org.firstinspires.ftc.teamcode.framework.command;

import static org.junit.Assert.*;

import java.util.Collections;
import java.util.Set;
import org.junit.Test;

public class CommandSchedulerTest {
    private static final class FakeSubsystem implements Subsystem {
        int periodicCalls;
        int stopCalls;
        @Override public void periodic() { periodicCalls++; }
        @Override public void stop() { stopCalls++; }
    }

    private static final class FakeCommand implements Command {
        final Set<Subsystem> requirements;
        int initializeCalls;
        int executeCalls;
        int interruptedEnds;
        int normalEnds;
        boolean finished;

        FakeCommand(Subsystem requirement) {
            requirements = Collections.singleton(requirement);
        }

        @Override public void initialize() { initializeCalls++; }
        @Override public void execute() { executeCalls++; }
        @Override public void end(boolean interrupted) {
            if (interrupted) interruptedEnds++; else normalEnds++;
        }
        @Override public boolean isFinished() { return finished; }
        @Override public Set<Subsystem> getRequirements() { return requirements; }
    }

    @Test public void scheduleRunsLifecycleAndReleasesRequirement() {
        CommandScheduler scheduler = new CommandScheduler();
        FakeSubsystem subsystem = new FakeSubsystem();
        FakeCommand command = new FakeCommand(subsystem);
        scheduler.registerSubsystem(subsystem);
        scheduler.schedule(command);
        assertEquals(1, command.initializeCalls);
        scheduler.run();
        assertEquals(1, command.executeCalls);
        command.finished = true;
        scheduler.run();
        assertEquals(1, command.normalEnds);
        assertFalse(scheduler.isScheduled(command));
    }

    @Test public void requirementConflictInterruptsExistingCommand() {
        CommandScheduler scheduler = new CommandScheduler();
        FakeSubsystem subsystem = new FakeSubsystem();
        FakeCommand first = new FakeCommand(subsystem);
        FakeCommand replacement = new FakeCommand(subsystem);
        scheduler.schedule(first);
        scheduler.schedule(replacement);
        assertEquals(1, first.interruptedEnds);
        assertFalse(scheduler.isScheduled(first));
        assertTrue(scheduler.isScheduled(replacement));
    }

    @Test public void defaultCommandRunsWhenSubsystemIsFree() {
        CommandScheduler scheduler = new CommandScheduler();
        FakeSubsystem subsystem = new FakeSubsystem();
        FakeCommand defaultCommand = new FakeCommand(subsystem);
        scheduler.registerSubsystem(subsystem);
        scheduler.setDefaultCommand(subsystem, defaultCommand);
        scheduler.run();
        assertEquals(1, defaultCommand.initializeCalls);
        assertEquals(1, defaultCommand.executeCalls);
        assertEquals(1, subsystem.periodicCalls);
    }

    @Test public void shutdownInterruptsCommandsStopsSubsystemsAndClearsState() {
        CommandScheduler scheduler = new CommandScheduler();
        FakeSubsystem subsystem = new FakeSubsystem();
        FakeCommand command = new FakeCommand(subsystem);
        scheduler.registerSubsystem(subsystem);
        scheduler.schedule(command);
        scheduler.shutdown();
        assertEquals(1, command.interruptedEnds);
        assertEquals(1, subsystem.stopCalls);
        assertFalse(scheduler.isScheduled(command));
        try {
            scheduler.run();
            fail("run after shutdown must fail");
        } catch (IllegalStateException expected) {
            assertEquals("scheduler is shut down", expected.getMessage());
        }
        assertEquals(0, command.executeCalls);
    }
}
```

- [ ] **Step 3: Run the focused test and verify the red state**

Run:

```bash
GRADLE_USER_HOME=/private/tmp/ftc20287-gradle \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:testDebugUnitTest --tests '*CommandSchedulerTest'
```

Expected: FAIL because `Subsystem`, `Command`, and `CommandScheduler` do not exist.

- [ ] **Step 4: Implement the contracts and scheduler**

Implement `Subsystem.java`:

```java
package org.firstinspires.ftc.teamcode.framework.command;

public interface Subsystem {
    default void periodic() {}
    void stop();
}
```

Implement `Command.java`:

```java
package org.firstinspires.ftc.teamcode.framework.command;

import java.util.Set;

public interface Command {
    default void initialize() {}
    void execute();
    default void end(boolean interrupted) {}
    default boolean isFinished() { return false; }
    Set<Subsystem> getRequirements();
}
```

Implement `CommandBase.java`:

```java
package org.firstinspires.ftc.teamcode.framework.command;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

public abstract class CommandBase implements Command {
    private final Set<Subsystem> requirements = new LinkedHashSet<>();

    protected final void addRequirements(Subsystem... subsystems) {
        if (subsystems == null) throw new IllegalArgumentException("subsystems cannot be null");
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) throw new IllegalArgumentException("subsystem cannot be null");
            requirements.add(subsystem);
        }
    }

    @Override public final Set<Subsystem> getRequirements() {
        return Collections.unmodifiableSet(requirements);
    }
}
```

Implement `CommandScheduler.java` with instance-owned state:

```java
package org.firstinspires.ftc.teamcode.framework.command;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public final class CommandScheduler {
    private final Set<Subsystem> subsystems = new LinkedHashSet<>();
    private final Set<Command> scheduled = new LinkedHashSet<>();
    private final Map<Subsystem, Command> owners = new LinkedHashMap<>();
    private final Map<Subsystem, Command> defaults = new LinkedHashMap<>();
    private boolean shutdown;

    public void registerSubsystem(Subsystem subsystem) {
        requireNonNull(subsystem, "subsystem");
        ensureActive();
        subsystems.add(subsystem);
    }

    public void setDefaultCommand(Subsystem subsystem, Command command) {
        requireNonNull(subsystem, "subsystem");
        requireNonNull(command, "command");
        ensureActive();
        if (!command.getRequirements().contains(subsystem)) {
            throw new IllegalArgumentException("default command must require its subsystem");
        }
        subsystems.add(subsystem);
        defaults.put(subsystem, command);
    }

    public void schedule(Command command) {
        requireNonNull(command, "command");
        ensureActive();
        if (scheduled.contains(command)) return;
        for (Subsystem requirement : command.getRequirements()) {
            requireNonNull(requirement, "command requirement");
            Command owner = owners.get(requirement);
            if (owner != null && owner != command) cancel(owner);
        }
        for (Subsystem requirement : command.getRequirements()) owners.put(requirement, command);
        scheduled.add(command);
        command.initialize();
    }

    public void run() {
        ensureActive();
        for (Subsystem subsystem : new ArrayList<>(subsystems)) subsystem.periodic();
        for (Map.Entry<Subsystem, Command> entry : new ArrayList<>(defaults.entrySet())) {
            if (!owners.containsKey(entry.getKey()) && !scheduled.contains(entry.getValue())) {
                schedule(entry.getValue());
            }
        }
        for (Command command : new ArrayList<>(scheduled)) {
            command.execute();
            if (command.isFinished()) finish(command, false);
        }
    }

    public void cancel(Command command) {
        if (command != null && scheduled.contains(command)) finish(command, true);
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduled)) finish(command, true);
    }

    public boolean isScheduled(Command command) {
        return scheduled.contains(command);
    }

    public void shutdown() {
        if (shutdown) return;
        cancelAll();
        for (Subsystem subsystem : new ArrayList<>(subsystems)) subsystem.stop();
        defaults.clear();
        owners.clear();
        subsystems.clear();
        shutdown = true;
    }

    private void finish(Command command, boolean interrupted) {
        scheduled.remove(command);
        owners.entrySet().removeIf(entry -> entry.getValue() == command);
        command.end(interrupted);
    }

    private void ensureActive() {
        if (shutdown) throw new IllegalStateException("scheduler is shut down");
    }

    private static void requireNonNull(Object value, String name) {
        if (value == null) throw new IllegalArgumentException(name + " cannot be null");
    }
}
```

- [ ] **Step 5: Run the focused test and verify the green state**

Run the Step 3 command again.

Expected: four `CommandSchedulerTest` tests PASS.

- [ ] **Step 6: Commit the command core**

```bash
git add -- TeamCode/build.gradle TeamCode/src/main/java/org/firstinspires/ftc/teamcode/framework TeamCode/src/test/java/org/firstinspires/ftc/teamcode/framework
git diff --cached --check
git commit -m "feat: add team command scheduler"
```

---

### Task 3: Add tested mecanum drive math

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/WheelPowers.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/MecanumMixer.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/drive/MecanumMixerTest.java`

**Interfaces:**
- Consumes: normalized robot-centric `forward`, `strafe`, and `turn` doubles.
- Produces: `MecanumMixer.mix(double, double, double): WheelPowers` with each output in `[-1, 1]`.

- [ ] **Step 1: Write failing mixer tests**

```java
package org.firstinspires.ftc.teamcode.drive;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

public class MecanumMixerTest {
    private static final double EPSILON = 1e-9;

    @Test public void zeroInputStopsAllWheels() {
        assertPowers(MecanumMixer.mix(0, 0, 0), 0, 0, 0, 0);
    }

    @Test public void forwardDrivesAllWheelsForward() {
        assertPowers(MecanumMixer.mix(1, 0, 0), 1, 1, 1, 1);
    }

    @Test public void strafeUsesMecanumSigns() {
        assertPowers(MecanumMixer.mix(0, 1, 0), 1, -1, -1, 1);
    }

    @Test public void turnUsesOppositeSideSigns() {
        assertPowers(MecanumMixer.mix(0, 0, 1), 1, -1, 1, -1);
    }

    @Test public void mixedInputIsNormalized() {
        WheelPowers powers = MecanumMixer.mix(1, 1, 1);
        assertPowers(powers, 1, -1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
        assertTrue(powers.maxMagnitude() <= 1.0);
    }

    private static void assertPowers(WheelPowers actual, double fl, double fr, double bl, double br) {
        assertEquals(fl, actual.frontLeft, EPSILON);
        assertEquals(fr, actual.frontRight, EPSILON);
        assertEquals(bl, actual.backLeft, EPSILON);
        assertEquals(br, actual.backRight, EPSILON);
    }
}
```

- [ ] **Step 2: Run the focused test and verify it fails**

```bash
GRADLE_USER_HOME=/private/tmp/ftc20287-gradle \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:testDebugUnitTest --tests '*MecanumMixerTest'
```

Expected: FAIL because `MecanumMixer` and `WheelPowers` do not exist.

- [ ] **Step 3: Implement immutable output and normalized mixing**

Create `WheelPowers.java`:

```java
package org.firstinspires.ftc.teamcode.drive;

public final class WheelPowers {
    public final double frontLeft;
    public final double frontRight;
    public final double backLeft;
    public final double backRight;

    public WheelPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public double maxMagnitude() {
        return Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight)));
    }
}
```

Create `MecanumMixer.java`:

```java
package org.firstinspires.ftc.teamcode.drive;

public final class MecanumMixer {
    private MecanumMixer() {}

    public static WheelPowers mix(double forward, double strafe, double turn) {
        double frontLeft = forward + strafe + turn;
        double frontRight = forward - strafe - turn;
        double backLeft = forward - strafe + turn;
        double backRight = forward + strafe - turn;
        double denominator = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight))));
        return new WheelPowers(frontLeft / denominator, frontRight / denominator,
                backLeft / denominator, backRight / denominator);
    }
}
```

- [ ] **Step 4: Run the focused test and verify it passes**

Run the Step 2 command again.

Expected: five `MecanumMixerTest` tests PASS.

- [ ] **Step 5: Commit drive math**

```bash
git add -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/WheelPowers.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/MecanumMixer.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/drive/MecanumMixerTest.java
git diff --cached --check
git commit -m "feat: add tested mecanum mixing"
```

---

### Task 4: Add centralized robot configuration and drivetrain hardware ownership

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotConfig.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotHardware.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/DriveSubsystem.java`

**Interfaces:**
- Consumes: FTC SDK `HardwareMap`, `DcMotorEx`, `DcMotorSimple.Direction`, `DcMotor.ZeroPowerBehavior`, and `DcMotor.RunMode`; `WheelPowers` from Task 3.
- Produces: `RobotHardware.createDrive(HardwareMap): DriveSubsystem`, `DriveSubsystem.drive(WheelPowers)`, and `DriveSubsystem.stop()`.

- [ ] **Step 1: Create explicit unverified configuration**

Create `RobotConfig.java`:

```java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class RobotConfig {
    private RobotConfig() {}

    // UNVERIFIED ON THE 2026-2027 ROBOT: confirm in the RC configuration before enabling movement.
    public static final String FRONT_LEFT_DRIVE = "front_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_LEFT_DRIVE = "back_left_drive";
    public static final String BACK_RIGHT_DRIVE = "back_right_drive";

    // UNVERIFIED ON THE CURRENT DRIVETRAIN: lift the robot and verify positive wheel directions.
    public static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double DRIVE_DEADBAND = 0.05;
    public static final double DRIVE_POWER_SCALE = 1.0;
}
```

- [ ] **Step 2: Implement safe, typed motor construction**

Create `RobotHardware.java`:

```java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.DriveSubsystem;

public final class RobotHardware {
    private RobotHardware() {}

    public static DriveSubsystem createDrive(HardwareMap hardwareMap) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap cannot be null");
        DcMotorEx frontLeft = motor(hardwareMap, RobotConfig.FRONT_LEFT_DRIVE,
                RobotConfig.LEFT_DRIVE_DIRECTION);
        DcMotorEx frontRight = motor(hardwareMap, RobotConfig.FRONT_RIGHT_DRIVE,
                RobotConfig.RIGHT_DRIVE_DIRECTION);
        DcMotorEx backLeft = motor(hardwareMap, RobotConfig.BACK_LEFT_DRIVE,
                RobotConfig.LEFT_DRIVE_DIRECTION);
        DcMotorEx backRight = motor(hardwareMap, RobotConfig.BACK_RIGHT_DRIVE,
                RobotConfig.RIGHT_DRIVE_DIRECTION);
        return new DriveSubsystem(frontLeft, frontRight, backLeft, backRight);
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name,
                                   DcMotorSimple.Direction direction) {
        DcMotorEx motor;
        try {
            motor = hardwareMap.get(DcMotorEx.class, name);
        } catch (RuntimeException exception) {
            throw new IllegalStateException("Missing or invalid DcMotorEx configuration: " + name,
                    exception);
        }
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
```

- [ ] **Step 3: Implement the drivetrain subsystem as the sole motor writer**

Create `DriveSubsystem.java`:

```java
package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.framework.command.Subsystem;

public final class DriveSubsystem implements Subsystem {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public DriveSubsystem(DcMotorEx frontLeft, DcMotorEx frontRight,
                          DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = requireMotor(frontLeft, "frontLeft");
        this.frontRight = requireMotor(frontRight, "frontRight");
        this.backLeft = requireMotor(backLeft, "backLeft");
        this.backRight = requireMotor(backRight, "backRight");
        stop();
    }

    public void drive(WheelPowers powers) {
        if (powers == null) throw new IllegalArgumentException("powers cannot be null");
        frontLeft.setPower(clamp(powers.frontLeft));
        frontRight.setPower(clamp(powers.frontRight));
        backLeft.setPower(clamp(powers.backLeft));
        backRight.setPower(clamp(powers.backRight));
    }

    @Override public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private static double clamp(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    private static DcMotorEx requireMotor(DcMotorEx motor, String name) {
        if (motor == null) throw new IllegalArgumentException(name + " cannot be null");
        return motor;
    }
}
```

- [ ] **Step 4: Compile the Android module**

Run:

```bash
GRADLE_USER_HOME=/private/tmp/ftc20287-gradle \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:compileDebugJavaWithJavac
```

Expected: `BUILD SUCCESSFUL`. No motor direction or hardware-map behavior is described as robot-verified.

- [ ] **Step 5: Commit centralized hardware ownership**

```bash
git add -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/DriveSubsystem.java
git diff --cached --check
git commit -m "feat: add centralized drivetrain hardware"
```

---

### Task 5: Compose the default drive command and TeleOp lifecycle

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/TeleopDriveCommand.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java`

**Interfaces:**
- Consumes: `DoubleSupplier` gamepad inputs, `MecanumMixer.mix`, `DriveSubsystem.drive`, `CommandScheduler`, and `RobotHardware.createDrive`.
- Produces: a supplier-driven nonterminating default command and Driver Station OpMode named `20827 Main TeleOp`.

- [ ] **Step 1: Implement the live-input default command**

Create `TeleopDriveCommand.java`:

```java
package org.firstinspires.ftc.teamcode.drive;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.framework.command.CommandBase;

public final class TeleopDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final double deadband;
    private final double scale;

    public TeleopDriveCommand(DriveSubsystem drive, DoubleSupplier forward,
                              DoubleSupplier strafe, DoubleSupplier turn,
                              double deadband, double scale) {
        if (drive == null || forward == null || strafe == null || turn == null) {
            throw new IllegalArgumentException("drive and input suppliers cannot be null");
        }
        if (deadband < 0.0 || deadband >= 1.0) {
            throw new IllegalArgumentException("deadband must be in [0, 1)");
        }
        if (scale < 0.0 || scale > 1.0) {
            throw new IllegalArgumentException("scale must be in [0, 1]");
        }
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.deadband = deadband;
        this.scale = scale;
        addRequirements(drive);
    }

    @Override public void execute() {
        double forwardValue = applyDeadband(forward.getAsDouble()) * scale;
        double strafeValue = applyDeadband(strafe.getAsDouble()) * scale;
        double turnValue = applyDeadband(turn.getAsDouble()) * scale;
        drive.drive(MecanumMixer.mix(forwardValue, strafeValue, turnValue));
    }

    @Override public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) <= deadband) return 0.0;
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }
}
```

- [ ] **Step 2: Implement the OpMode composition root with guaranteed cleanup**

Create `MainTeleOp.java`:

```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.framework.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "20827 Main TeleOp", group = "BIOBUZZ")
public final class MainTeleOp extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        CommandScheduler scheduler = new CommandScheduler();
        try {
            DriveSubsystem drive = RobotHardware.createDrive(hardwareMap);
            scheduler.registerSubsystem(drive);
            scheduler.setDefaultCommand(drive, new TeleopDriveCommand(
                    drive,
                    () -> -gamepad1.left_stick_y,
                    () -> gamepad1.left_stick_x,
                    () -> gamepad1.right_stick_x,
                    RobotConfig.DRIVE_DEADBAND,
                    RobotConfig.DRIVE_POWER_SCALE));

            telemetry.addLine("Framework ready; drivetrain configuration is not robot-verified.");
            telemetry.addLine("Lift the robot before the first direction test.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                scheduler.run();
                telemetry.addData("Status", "Running");
                telemetry.update();
                idle();
            }
        } finally {
            scheduler.shutdown();
        }
    }
}
```

- [ ] **Step 3: Run all local unit tests and compile the OpMode**

```bash
GRADLE_USER_HOME=/private/tmp/ftc20287-gradle \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac
```

Expected: nine unit tests PASS and `BUILD SUCCESSFUL`.

- [ ] **Step 4: Commit the first runnable composition**

```bash
git add -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/TeleopDriveCommand.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTeleOp.java
git diff --cached --check
git commit -m "feat: add BIOBUZZ mecanum TeleOp"
```

---

### Task 6: Document, verify, and classify the software result

**Files:**
- Create: `TeamCode/README.md`
- Modify only if required by a real FTCKB hard rule: files created in Tasks 2–5.

**Interfaces:**
- Consumes: all framework entry points, FTCKB check/verify tools, and Gradle tasks.
- Produces: teammate-facing setup and a software-verified, hardware-unverified handoff.

- [ ] **Step 1: Write the TeamCode handoff**

Create `TeamCode/README.md` with these exact sections and facts:

```markdown
# FTC-20827 BioBuzz TeamCode

This module contains the 2026–2027 team-owned command framework. It uses FTC SDK 12.0 and does not depend on FTCLib.

## Structure

- `framework/command`: command lifecycle, subsystem requirements, defaults, and shutdown
- `drive`: pure mecanum math, drivetrain ownership, and the default TeleOp drive command
- `robot`: centralized hardware names and initialization
- `opmodes`: Driver Station entry points

## Before running on a robot

1. Match the four names in `RobotConfig` to the Robot Controller configuration.
2. Lift the robot so every wheel is clear of the floor.
3. Verify each motor direction at low commanded power.
4. Verify forward, strafe, and turn controls independently.
5. Verify STOP immediately sets all drive powers to zero.
6. Record the verified names and directions in the review/commit that changes them.

Current status: software validation may pass, but hardware names, directions, controls, and emergency-stop behavior are unverified until the checklist above is completed on the current robot.

## Developer checks

Run FTCKB resolution before editing and check/verify after editing. Then run `:TeamCode:testDebugUnitTest` and `:TeamCode:assembleDebug` with JDK 21 and a configured Android SDK.
```

- [ ] **Step 2: Run the complete TeamCode software checks**

```bash
GRADLE_USER_HOME=/private/tmp/ftc20287-gradle \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Expected: nine unit tests PASS and `BUILD SUCCESSFUL`. If Android SDK lookup fails because `local.properties` is absent, rerun with the machine's existing `ANDROID_HOME` environment value; do not create or commit a personal SDK path.

- [ ] **Step 3: Run the FTCKB diff check**

```bash
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
python3 tools/FTC-Knowledge-Bank/.agents/skills/ftckb-integrate/scripts/project.py check --project .
```

Expected: kernel JSON with `ok=true`, or a documented `ok=false` with exact hard-rule IDs. Do not add irrelevant Limelight calls to silence a false positive; classify any known broad Java matcher separately.

- [ ] **Step 4: Run full FTCKB integration verification**

```bash
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
python3 tools/FTC-Knowledge-Bank/.agents/skills/ftckb-integrate/scripts/verify.py --project .
```

Expected: `installationOk=true`. Report `projectCheck.ok` independently, including all soft advice and hard failures.

- [ ] **Step 5: Review scope and commit documentation or compliance fixes**

```bash
git diff --check
git status --short --branch
git diff --stat HEAD
git add -- TeamCode/README.md
git diff --cached --check
git diff --cached --stat
git commit -m "docs: add BIOBUZZ TeamCode validation guide"
```

If FTCKB required real code changes, stage their exact paths in the same commit only after rerunning Steps 2–4; otherwise stage only `TeamCode/README.md`.

- [ ] **Step 6: Produce the final evidence summary**

Report:

```text
FTCKB pin: 105c8e47ddee734365af50dd2305caa13646ab21
FTCKB installation: pass/fail
FTCKB project check: pass/fail with rule IDs
Unit tests: passed/failed count
TeamCode assembleDebug: pass/fail
Android Studio Sync: not tested unless actually performed
Control Hub deployment: not tested
Physical robot configuration/directions/STOP: unverified until team checklist completion
Working tree: exact git status
```

Do not push unless the user separately authorizes publication.
