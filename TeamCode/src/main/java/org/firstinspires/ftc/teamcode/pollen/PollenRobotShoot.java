/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

// Hardware conventions and launcher settings adapted from the goBILDA StarterBot sample.

package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Shoot from the end wall, then translate diagonally without changing heading. */
@Autonomous(name = "Pollen ROBOT Shoot + Park", group = "Pollen", preselectTeleOp = "PollenBot")
public final class PollenRobotShoot extends OpMode {
    private static final String BUILD = "pollen-prep-v3.4-angle";
    private static PollenShootTuning.Snapshot remembered;
    private final DcMotor[] drive = new DcMotor[4];
    private DcMotorEx launcher;
    private DcMotor intake, agitator;
    private final ElapsedTime clock = new ElapsedTime();
    private PollenShootTuning tuning;
    private PollenSequence sequence;
    private String blocked, fallback;
    private boolean shooterAvailable;

    @Override public final void init() {
        blocked = fallback = null;
        sequence = null;
        shooterAvailable = true;
        tuning = new PollenShootTuning(remembered);
        try {
            mapDrive();
            try {
                mapShooter();
            } catch (RuntimeException ex) {
                if (!zeroShooter()) throw new IllegalStateException("Cannot confirm shooter stop", ex);
                shooterAvailable = false;
                fallback = "NO SHOT: " + ex + "; same selected diagonal departure only";
            }
            tuning.sequence(shooterAvailable); // Validate defaults before reporting ready.
        } catch (RuntimeException ex) {
            blocked = ex.toString();
            zeroHardware();
        }
        init_loop();
    }

    private void mapDrive() {
        for (int i = 0; i < drive.length; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
            drive[i].setPower(0);
            drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void mapShooter() {
        // Team's BioBuzzStarterbotOpMode / LauncherTestOpMode mapping and goBILDA PIDF.
        launcher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        launcher.setPower(0);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator = hardwareMap.get(DcMotor.class, "agitator");
        agitator.setPower(0);
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override public final void init_loop() {
        if (gamepad1 != null) tuning.update(gamepad1.dpad_up, gamepad1.dpad_down,
                gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.triangle);
        remembered = tuning.snapshot();
        showSettings();
        telemetry.addData("Controls", "D-pad UP/DOWN selects; LEFT/RIGHT edits; tap then release");
        telemetry.addData("Reset", "Triangle=defaults (2 ft)");
        telemetry.addData("Status", blocked != null ? "BLOCKED: " + blocked
                : fallback != null ? "READY FOR DEPARTURE ONLY - SHOOTER FAILED" : "READY - no movement until START");
        telemetry.addData("Memory", "Settings kept until app restart; start delay resets every INIT");
        telemetry.update();
    }

    private void showSettings() {
        telemetry.addData("Build", BUILD);
        telemetry.addData("Mode", "ROBOT Shoot + Park - real motors on START");
        telemetry.addData("Selections", tuning.locked() ? "LOCKED at START" : "INIT editing");
        telemetry.addData("Start pose", "End wall, square to wall, facing your hive; same controls for either alliance");
        for (PollenShootTuning.Field f : PollenShootTuning.Field.values()) {
            telemetry.addData((f == tuning.selected() ? "> " : "  ") + f.label, "%.2f", tuning.value(f));
        }
        telemetry.addData("Travel", "%.2f ft / %.2f estimated seconds; angle %+.2f deg (+left/-right); NO TURN",
                tuning.value(DISTANCE_CM) / 30.48, tuning.driveSeconds(), tuning.value(TRAVEL_ANGLE));
        telemetry.addData("Worst case / limit", "%.2f / %.0f seconds", tuning.worstCaseSeconds(), PollenShootTuning.DEADLINE);
        telemetry.addData("Calibration", "Factor 129 fitted near 25.87 deg; remeasure distance when angle changes");
        if (fallback != null) telemetry.addData("SHOOTER INIT FAILED", fallback);
        if (!tuning.notice().isEmpty()) telemetry.addData("Settings notice", tuning.notice());
    }

    @Override public final void start() {
        tuning.lock();
        clock.reset();
        if (blocked != null) { zeroHardware(); requestOpModeStop(); return; }
        try {
            sequence = tuning.sequence(shooterAvailable);
            sequence.start(0);
        } catch (RuntimeException ex) {
            fail(ex);
        }
    }

    @Override public final void loop() {
        if (blocked != null || sequence == null) return;
        try {
            double now = clock.seconds();
            double velocity = shooterAvailable ? launcher.getVelocity() : 0;
            PollenSequence.Output out = sequence.tick(now, velocity, false);
            double[] powers = PollenSequence.wheelPowers(out);
            // Stop the shooting mechanisms BEFORE the first parking drive command.
            if (shooterAvailable) {
                launcher.setVelocity(out.launcherVelocity);
                intake.setPower(out.feederPower * tuning.value(FEED_POWER));
                agitator.setPower(out.feederPower * tuning.value(FEED_POWER));
            }
            for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
            showSettings();
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Elapsed", "%.2f s", now);
            telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s", out.launcherVelocity, velocity);
            telemetry.addData("Intake + feeder command", "%.2f", out.feederPower * tuning.value(FEED_POWER));
            telemetry.addData("Commanded feed time", "%.2f s", out.activeFeedSeconds);
            telemetry.addData("FL FR BL BR", "%.3f %.3f %.3f %.3f", powers[0], powers[1], powers[2], powers[3]);
            if (!sequence.warning().isEmpty()) telemetry.addData("Warning", sequence.warning());
            telemetry.update();
        } catch (RuntimeException ex) {
            fail(ex);
        }
    }

    private void fail(RuntimeException ex) {
        blocked = ex.toString();
        if (sequence != null) sequence.stop("HARDWARE ERROR");
        zeroHardware();
        telemetry.addData("ERROR - ALL OUTPUTS STOPPED", blocked);
        telemetry.update();
        requestOpModeStop();
    }

    @Override public final void stop() {
        if (tuning != null) tuning.lock();
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
    }

    private void zeroHardware() {
        zeroShooter();
        for (DcMotor motor : drive) zero(motor);
    }

    private boolean zeroShooter() {
        boolean ok = zero(launcher);
        ok &= zero(intake);
        ok &= zero(agitator);
        return ok;
    }

    private boolean zero(DcMotor motor) {
        if (motor == null) return true;
        try { motor.setPower(0); return true; }
        catch (RuntimeException ex) { telemetry.addData("Stop motor error", ex.toString()); return false; }
    }
}
