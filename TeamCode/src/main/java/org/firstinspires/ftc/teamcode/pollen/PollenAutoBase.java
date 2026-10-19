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
    private String blocked;
    private double simulatedVelocity;
    private double simulatedSpinupStarted = Double.NaN;

    @Override public final void init() {
        try {
            sequence = new PollenSequence(shoot() ? PollenConfig.shootRoute() : PollenConfig.parkRoute(),
                    PollenConfig.LAUNCHER_TARGET, PollenConfig.LAUNCHER_MIN,
                    PollenConfig.SPINUP_TIMEOUT, PollenConfig.SPEED_STABLE_TIME,
                    PollenConfig.TOTAL_TIME_LIMIT);
            if (!bench()) {
                if (!PollenConfig.ENABLE_ROBOT_PARK_TEST
                        || (shoot() && !PollenConfig.ENABLE_ROBOT_SHOOT_TEST)) {
                    blocked = "Robot tests disabled in PollenConfig; use BENCH tonight";
                } else {
                    mapHardware();
                }
            }
        } catch (RuntimeException ex) {
            blocked = ex.getMessage() == null ? ex.toString() : ex.getMessage();
            zeroHardware();
        }
        init_loop();
    }

    private void mapHardware() {
        for (int i = 0; i < drive.length; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
            drive[i].setPower(0);
            drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Park-only does not require any shooting/intake devices or an Expansion Hub.
        if (!shoot()) return;
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
        telemetry.addData("Build", "pollen-prep-v1");
        telemetry.addData("Mode", bench() ? "BENCH - NO HARDWARE COMMANDS" : "ROBOT TEST");
        telemetry.addData("Route", shoot() ? "move / shoot / park" : "move / park");
        telemetry.addData("Status", blocked == null ? "READY - waiting for START" : "BLOCKED: " + blocked);
        telemetry.addData("Calibration", "Example route; actual field path still needs tuning");
        telemetry.update();
    }

    @Override public final void start() {
        clock.reset();
        simulatedVelocity = 0;
        simulatedSpinupStarted = Double.NaN;
        if (blocked == null) sequence.start(0);
    }

    @Override public final void loop() {
        if (blocked != null) { init_loop(); return; }
        try {
            double now = clock.seconds();
            double velocity = bench() ? simulatedVelocity : (launcher == null ? 0 : launcher.getVelocity());
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
                if (launcher != null) {
                    launcher.setVelocity(out.launcherVelocity);
                    windmill.setPower(out.feederPower);
                    intake.setPower(0.5 * out.feederPower);
                    leftIntake.setPower(0.5 * out.feederPower);
                    rightIntake.setPower(0.5 * out.feederPower);
                }
            }
            telemetry.addData("Build", "pollen-prep-v1");
            telemetry.addData("Mode", bench() ? "BENCH - simulated, no hardware commands" : "ROBOT TEST");
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Elapsed seconds", "%.2f", now);
            telemetry.addData("Requested FL FR BL BR", "%.2f %.2f %.2f %.2f",
                    powers[0], powers[1], powers[2], powers[3]);
            telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s", out.launcherVelocity, velocity);
            telemetry.addData("Feeder request", out.feederPower);
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
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
    }

    /** Best effort per device: one failed connection must not prevent stopping the others. */
    private void zeroHardware() {
        for (DcMotor motor : drive) zero(motor);
        zero(launcher);
        zero(intake);
        zero(windmill);
        zero(leftIntake);
        zero(rightIntake);
    }

    private void zero(DcMotor motor) {
        if (motor != null) {
            try { motor.setPower(0); }
            catch (RuntimeException ex) { telemetry.addData("Stop motor error", ex.toString()); }
        }
    }

    private void zero(CRServo servo) {
        if (servo != null) {
            try { servo.setPower(0); }
            catch (RuntimeException ex) { telemetry.addData("Stop servo error", ex.toString()); }
        }
    }
}
