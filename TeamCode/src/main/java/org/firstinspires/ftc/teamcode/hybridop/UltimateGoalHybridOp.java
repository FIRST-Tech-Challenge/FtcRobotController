package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.sequences.ShootActionSequence;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.HybridOpExecutor;

@TeleOp(name="Ultimate Goal Tele Op")
public class UltimateGoalHybridOp extends UltimateGoalHardware implements HybridOp {

    double spinnerPower = 1;
    double spinner_increment = 0.05;

    long prevTime = System.nanoTime();
    int prevPos;

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
        telemetry.addData("spinner power", String.format("%.2f", spinnerPower));

        // region Gamepad 1

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_up)) {
            double newPower = spinnerPower + spinner_increment;
            if (newPower <= 1) {
                spinnerPower = newPower;
            }
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_down)) {
            double newPower = spinnerPower - spinner_increment;
            if (newPower >= 0) {
                spinnerPower = newPower;
            }
        }

        if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.b)) {
            shooterLeft.setPower(-spinnerPower);
            shooterRight.setPower(spinnerPower);
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.x)) {
            this.hybridOpExecutor.executeActionSequence(new ShootActionSequence(), true, true);
        }

        if (gamepad1.a) {
            escalator.setPower(1);
        } else {
            escalator.setPower(0);
        }

        if (gamepad1.y) {
            collector.setPower(1);
        } else {
            collector.setPower(0);
        }

        //endregion
    }

    @Override
    public void init() {
        super.init();
        this.initializeForHybridOp(this);
    }

    @Override
    public void run_loop() {
        int cpos = shooterRight.getCurrentPosition();
        long ctime = System.currentTimeMillis()*60*1000;
        double rpm  = ((cpos/40.0-prevPos/40.0)/(ctime-prevTime));
        telemetry.addData("Shooter pos", cpos);
        telemetry.addData("Shooter rpm", rpm);
        prevPos = cpos;
        prevTime = ctime;

        this.gamepadActions.telemetry(telemetry);
    }
}
