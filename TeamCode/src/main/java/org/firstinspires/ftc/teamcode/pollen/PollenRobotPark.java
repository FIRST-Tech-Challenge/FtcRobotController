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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Option A: select delay, power and time during INIT, then drive forward and stop. */
@Autonomous(name = "Pollen ROBOT Park", group = "Pollen", preselectTeleOp = "PollenBot")
public final class PollenRobotPark extends OpMode {
    private static PollenParkTuning.Snapshot remembered;
    private final DcMotor[] drive = new DcMotor[4];
    private final ElapsedTime clock = new ElapsedTime();
    private PollenParkTuning tuning;
    private PollenSequence sequence;
    private String blocked;

    @Override public void init() {
        blocked = null;
        tuning = null;
        sequence = null;
        try {
            for (int i = 0; i < drive.length; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
                drive[i].setPower(0);
                drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            tuning = new PollenParkTuning(PollenConfig.PARK_FORWARD_POWER,
                    PollenConfig.PARK_FORWARD_SECONDS, PollenConfig.START_DELAY_SECONDS,
                    PollenConfig.TOTAL_TIME_LIMIT, PollenConfig.PARK_HOLD_SECONDS, remembered);
            sequence = selectedSequence();
        } catch (RuntimeException ex) {
            blocked = ex.toString();
            zeroHardware();
        }
        init_loop();
    }

    private PollenSequence selectedSequence() {
        return PollenSequence.park(PollenConfig.parkRoute(tuning.seconds(), tuning.power()),
                PollenConfig.TOTAL_TIME_LIMIT, tuning.delaySeconds());
    }

    @Override public void init_loop() {
        if (tuning != null && gamepad1 != null) {
            tuning.update(gamepad1.dpad_up, gamepad1.dpad_down,
                    gamepad1.dpad_left, gamepad1.dpad_right,
                    gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle);
            remembered = tuning.snapshot();
        }
        showSettings();
        telemetry.addData("D-pad", "UP/DOWN delay +/-0.5s; RIGHT/LEFT drive +/-0.05s");
        telemetry.addData("L1 / R1", "Decrease / increase power by 0.05");
        telemetry.addData("Triangle", "Reset power, time, delay to code defaults");
        telemetry.addData("Memory", "Power/time kept until app restarts; delay resets each INIT");
        telemetry.addData("Placement", "Set starting position and angle by hand before START");
        telemetry.addData("Status", blocked == null ? "READY - waiting for START" : "BLOCKED: " + blocked);
        telemetry.update();
    }

    private void showSettings() {
        telemetry.addData("Build", "pollen-prep-v3.3-clean");
        telemetry.addData("Mode", "ROBOT Park - real motors on START");
        if (tuning == null) return;
        telemetry.addData("Park power / drive seconds", "%.2f / %.2f s", tuning.power(), tuning.seconds());
        telemetry.addData("Start delay / maximum", "%.1f / %.1f seconds",
                tuning.delaySeconds(), tuning.maximumDelaySeconds());
        telemetry.addData("Selection", tuning.locked() ? "LOCKED" : "Adjust on Gamepad 1 before START");
        telemetry.addData("Keep these values", "PARK_FORWARD_POWER=%.2f; PARK_FORWARD_SECONDS=%.2f",
                tuning.power(), tuning.seconds());
    }

    @Override public void start() {
        clock.reset();
        if (tuning != null) tuning.lock();
        if (blocked != null) { zeroHardware(); requestOpModeStop(); return; }
        try {
            sequence = selectedSequence();
            sequence.start(0);
        } catch (RuntimeException ex) { fail(ex); }
    }

    @Override public void loop() {
        if (blocked != null || sequence == null) return;
        try {
            double now = clock.seconds();
            PollenSequence.Output out = sequence.tick(now, 0, false);
            double[] powers = PollenSequence.wheelPowers(out);
            for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
            showSettings();
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Delay remaining", "%.1f seconds",
                    out.finished ? 0 : Math.max(0, tuning.delaySeconds() - now));
            telemetry.addData("Elapsed seconds", "%.2f", now);
            telemetry.addData("FL FR BL BR", "%.2f %.2f %.2f %.2f",
                    powers[0], powers[1], powers[2], powers[3]);
            telemetry.update();
        } catch (RuntimeException ex) { fail(ex); }
    }

    private void fail(RuntimeException ex) {
        blocked = ex.toString();
        if (sequence != null) sequence.stop("HARDWARE ERROR");
        zeroHardware();
        telemetry.addData("ERROR - stopped", blocked);
        telemetry.update();
        requestOpModeStop();
    }

    @Override public void stop() {
        if (tuning != null) tuning.lock();
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
    }

    private void zeroHardware() {
        for (DcMotor motor : drive) {
            if (motor == null) continue;
            try { motor.setPower(0); }
            catch (RuntimeException ex) { telemetry.addData("Stop motor error", ex.toString()); }
        }
    }
}
