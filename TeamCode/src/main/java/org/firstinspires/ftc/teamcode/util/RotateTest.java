package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;

@TeleOp(name="Rotate test")
public class RotateTest extends UltimateGoalHardware implements HybridOp {

    double power = 0;

    @Override
    public void init() {
        super.init();
        this.initializeForHybridOp(this);
    }

    @Override
    public void run_loop() {

    }

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_up)) {
            power += 0.005;
        } else if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_down)) {
            power -= 0.005;
        }
        omniDrive.rotateRight(power);
        telemetry.addData("rotate speed", power);
    }
}
