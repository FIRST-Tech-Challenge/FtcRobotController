package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;

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

        omniDrive.dpadMove(gamepad1, 1f, false);

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.TWO, GamepadActions.GamepadButtons.dpad_up)) {
            double newPower = spinnerPower + spinner_increment;
            if (newPower <= 1) {
                spinnerPower = newPower;
            }
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.TWO, GamepadActions.GamepadButtons.dpad_right)) {
            double newPower = spinnerPower + .005;
            if (newPower <= 1) {
                spinnerPower = newPower;
            }
        }



        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.TWO, GamepadActions.GamepadButtons.dpad_down)) {
            double newPower = spinnerPower - spinner_increment;
            if (newPower >= 0) {
                spinnerPower = newPower;
            }
        }

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.TWO, GamepadActions.GamepadButtons.dpad_left)) {
            double newPower = spinnerPower - .005;
            if (newPower >= 0) {
                spinnerPower = newPower;
            }
        }

        if (gamepadActions.isToggled(GamepadActions.GamepadType.TWO, GamepadActions.GamepadButtons.b)) {
            shooter.setPower(-spinnerPower);
        } else {
            shooter.setPower(0);
        }

        if (gamepad2.a) {
            escalator.setPower(1);
        } else {
            escalator.setPower(0);
        }

        if (gamepad2.y) {
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
        this.localizer.telemetry(telemetry);
    }
}
