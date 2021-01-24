package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

@TeleOp(name = "Ultimate Goal TeleOp")
public class UltimateGoalHybridOp extends UltimateGoalHardware implements HybridOp {

    float omniDrivePower = 0.5f;
//    double spinnerPower = 1;
//    double largeSpinnerIncrement = 0.05;
//    double smallSpinnerIncrement = 0.005;
    boolean slowMode = false;
    float slowModeMultiplier = 0.25f;

    long prevTime = System.currentTimeMillis();
    int prevPos = 0;

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {

        // region slowmode {...}
        slowMode = gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_left);

        if (shooter.getPower() > 0) {
            slowMode = true;
        }
        // endregion

        omniDrive.dpadMove(gamepad1, slowMode ? omniDrivePower * slowModeMultiplier : omniDrivePower, false);
        //region shooter

        if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.b)) {
            shooter.setPower(SHOOTER_POWER);
        } else {
            shooter.setPower(0);
        }
        // endregion

        if (gamepad1.b) {
            escalator.setPower(1);
        } else {
            escalator.setPower(0);
        }

        if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a)) {
            collector.setPower(1);
        } else {
            collector.setPower(0);
        }

        // region wobbleGoal {...}
        if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.x)) {
            wobbleGoalHolder.setTargetPosition(1);
        } else {
            wobbleGoalHolder.setTargetPosition(0);
        }

        if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_right)) {
            wobbleServo.setPosition(1);
        } else {
            wobbleServo.setPosition(0);
        }
        // endregion


        // Collector should not run if shooter is running
        if (shooter.getPower() > 0) {
            collector.setPower(0);
        }

    }

    @Override
    public void init() {
        super.init();
        this.initializeForHybridOp(this);
    }

    @Override
    public void run_loop() {
        long current_time = System.currentTimeMillis();
        int current_pos = shooter.getCurrentPosition();
        int deltaPos = current_pos - prevPos;
        long deltaTime = current_time - prevTime;
        prevPos = current_pos;
        prevTime = current_time;
        double rpm = (deltaPos/28.0) / (deltaTime) * (1000*60);
        telemetry.addData("RPM", rpm);
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("BL", backLeft.getCurrentPosition());
        telemetry.addData("BR", backRight.getCurrentPosition());
    }
}
