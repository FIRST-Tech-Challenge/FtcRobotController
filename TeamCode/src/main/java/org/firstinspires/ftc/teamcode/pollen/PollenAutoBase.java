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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/** The same sequencer drives bench telemetry and physical hardware. No blocking loops. */
public abstract class PollenAutoBase extends OpMode {
    protected abstract boolean bench();
    protected abstract boolean shoot();
    protected boolean simulateLauncherFailure() { return false; }

    private final DcMotor[] drive = new DcMotor[4];
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo windmill, leftIntake, rightIntake;
    private final ElapsedTime clock = new ElapsedTime();
    private PollenSequence sequence;
    private PollenSequence.Step[] route;
    private PollenStartDelay startDelay;
    private String blocked;
    private String fallback;
    private boolean activeShoot;
    private double simulatedVelocity;
    private double simulatedSpinupStarted = Double.NaN;

    @Override public final void init() {
        blocked = fallback = null;
        startDelay = null;
        sequence = null;
        activeShoot = shoot();
        try {
            if (!bench()) {
                mapDrive(); // Drive failure must block both routes.
                if (activeShoot) {
                    try {
                        mapShooter();
                    } catch (RuntimeException ex) {
                        // Only safe before movement: parkRoute starts at the original start.
                        if (!zeroShooter()) {
                            throw new IllegalStateException("Shooter init failed and stop could not be confirmed", ex);
                        }
                        activeShoot = false;
                        fallback = "PARK ONLY - shooter init failed: " + ex;
                    }
                }
            }
            route = activeShoot ? PollenConfig.shootRoute() : PollenConfig.parkRoute();
            // Park has a selectable delay. Shoot retains its configured route and delay.
            if (!shoot()) {
                startDelay = new PollenStartDelay(PollenConfig.START_DELAY_SECONDS,
                        PollenSequence.worstCaseDuration(route, PollenConfig.SPINUP_TIMEOUT),
                        PollenConfig.TOTAL_TIME_LIMIT);
            }
            sequence = newSequence(selectedDelay());
        } catch (RuntimeException ex) {
            blocked = ex.getMessage() == null ? ex.toString() : ex.getMessage();
            zeroHardware();
        }
        init_loop();
    }

    private double selectedDelay() {
        return startDelay == null ? PollenConfig.START_DELAY_SECONDS : startDelay.delaySeconds();
    }

    private PollenSequence newSequence(double delay) {
        return new PollenSequence(route, PollenConfig.LAUNCHER_TARGET, PollenConfig.LAUNCHER_MIN,
                PollenConfig.SPINUP_TIMEOUT, PollenConfig.SPEED_STABLE_TIME,
                PollenConfig.TOTAL_TIME_LIMIT, delay);
    }

    private void showDelay() {
        if (startDelay == null) return;
        telemetry.addData("Start delay / maximum", "%.1f / %.1f seconds",
                startDelay.delaySeconds(), startDelay.maximumDelaySeconds());
        telemetry.addData("Selection", startDelay.locked() ? "LOCKED" : "Adjust on Gamepad 1 before START");
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
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setPower(0);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windmill = hardwareMap.get(CRServo.class, "windmillServo");
        windmill.setPower(0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake = hardwareMap.get(CRServo.class, "leftIntakeServo");
        leftIntake.setPower(0);
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake = hardwareMap.get(CRServo.class, "rightIntakeServo");
        rightIntake.setPower(0);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override public final void init_loop() {
        if (startDelay != null && gamepad1 != null) {
            startDelay.update(gamepad1.dpad_up, gamepad1.dpad_down);
        }
        telemetry.addData("Build", "pollen-prep-v3");
        telemetry.addData("Mode", bench() ? "BENCH - NO HARDWARE COMMANDS" : "ROBOT TEST");
        telemetry.addData("Active route", activeShoot ? "move / shoot / park" : "move / park");
        telemetry.addData("Start delay seconds", selectedDelay());
        showDelay();
        if (startDelay != null && !startDelay.locked()) {
            telemetry.addData("D-pad", "UP/DOWN=delay +/-0.5s per press");
            telemetry.addData("Placement", "Set starting position and angle by hand before START.");
        }
        if (fallback != null) telemetry.addData("FALLBACK", fallback);
        telemetry.addData("Status", blocked == null ? "READY - waiting for START" : "BLOCKED: " + blocked);
        telemetry.addData("Calibration", "Example route; actual field path still needs tuning");
        telemetry.update();
    }

    @Override public final void start() {
        clock.reset();
        simulatedVelocity = 0;
        simulatedSpinupStarted = Double.NaN;
        if (startDelay != null) startDelay.lock();
        if (blocked == null) {
            try {
                sequence = newSequence(selectedDelay()); // Snapshot of INIT selections.
                sequence.start(0);
            } catch (RuntimeException ex) {
                blocked = ex.toString();
                zeroHardware();
                requestOpModeStop();
            }
        }
    }

    @Override public final void loop() {
        if (blocked != null) { init_loop(); return; }
        try {
            double now = clock.seconds();
            double velocity = bench() ? simulatedVelocity : (activeShoot ? launcher.getVelocity() : 0);
            PollenSequence.Output out = sequence.tick(now, velocity, false);
            double[] powers = PollenSequence.wheelPowers(out);
            if (bench()) {
                if (out.launcherVelocity == 0) {
                    simulatedSpinupStarted = Double.NaN;
                    simulatedVelocity = 0;
                } else {
                    if (Double.isNaN(simulatedSpinupStarted)) simulatedSpinupStarted = now;
                    simulatedVelocity = !simulateLauncherFailure() && now - simulatedSpinupStarted >= 1
                            ? PollenConfig.LAUNCHER_TARGET : 0;
                }
            } else {
                for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
                if (activeShoot) {
                    launcher.setVelocity(out.launcherVelocity);
                    windmill.setPower(out.feederPower);
                    intake.setPower(0.5 * out.feederPower);
                    leftIntake.setPower(0.5 * out.feederPower);
                    rightIntake.setPower(0.5 * out.feederPower);
                }
            }
            telemetry.addData("Build", "pollen-prep-v3");
            telemetry.addData("Mode", bench() ? "BENCH - simulated, no hardware commands" : "ROBOT TEST");
            showDelay();
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Delay remaining", "%.1f seconds",
                    out.finished ? 0 : Math.max(0, selectedDelay() - now));
            telemetry.addData("Elapsed seconds", "%.2f", now);
            telemetry.addData("Requested FL FR BL BR", "%.2f %.2f %.2f %.2f",
                    powers[0], powers[1], powers[2], powers[3]);
            telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s", out.launcherVelocity, velocity);
            telemetry.addData("Feeder request", out.feederPower);
            telemetry.addData("Commanded feed seconds", "%.2f", out.activeFeedSeconds);
            if (fallback != null) telemetry.addData("FALLBACK", fallback);
            if (!sequence.warning().isEmpty()) telemetry.addData("Warning", sequence.warning());
            telemetry.update();
            // Remain in DONE with zero commands so the driver can read the result and press STOP.
        } catch (RuntimeException ex) {
            blocked = ex.getMessage() == null ? ex.toString() : ex.getMessage();
            sequence.stop("HARDWARE ERROR");
            zeroHardware();
            telemetry.addData("ERROR - stopped", blocked);
            telemetry.update();
            requestOpModeStop();
        }
    }

    @Override public final void stop() {
        if (startDelay != null) startDelay.lock();
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
    }

    /** Best effort per device: one failed connection must not prevent stopping the others. */
    private void zeroHardware() {
        for (DcMotor motor : drive) zero(motor);
        zeroShooter();
    }

    private boolean zeroShooter() {
        // &= intentionally evaluates every device, including after one stop fails.
        boolean stopped = zero(launcher);
        stopped &= zero(intake);
        stopped &= zero(windmill);
        stopped &= zero(leftIntake);
        stopped &= zero(rightIntake);
        return stopped;
    }

    private boolean zero(DcMotor motor) {
        if (motor != null) {
            try { motor.setPower(0); }
            catch (RuntimeException ex) {
                telemetry.addData("Stop motor error", ex.toString());
                return false;
            }
        }
        return true;
    }

    private boolean zero(CRServo servo) {
        if (servo != null) {
            try { servo.setPower(0); }
            catch (RuntimeException ex) {
                telemetry.addData("Stop servo error", ex.toString());
                return false;
            }
        }
        return true;
    }
}
