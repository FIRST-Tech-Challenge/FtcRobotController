package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

@TeleOp(name = "Ultimate Goal TeleOp")
public class UltimateGoalHybridOp extends UltimateGoalHardware implements HybridOp {

    float omniDrivePower = 1f;
//    double spinnerPower = 1;
//    double largeSpinnerIncrement = 0.05;
//    double smallSpinnerIncrement = 0.005;
    boolean slowMode = false;
    float slowModeMultiplier = 0.5f;



    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {

        // region slowmode {...}

        // Slowmode when shooter running or bumper_left toggled
        slowMode = gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_left) || shooter.getPower() > 0;


        // endregion

        omniDrive.dpadMove(gamepad1, slowMode ? omniDrivePower * slowModeMultiplier : omniDrivePower, false);

        this.setShooterEnabled(gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.y));
        // endregion

        if (gamepad1.b && this.canShoot()) {
            escalator.setPower(1);
        } else {
            escalator.setPower(0);
        }

        if (gamepad1.start) {
            collector.setPower(-1);
        } else if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.a)) {
            collector.setPower(1);
        } else {
            collector.setPower(0);
        }

        // region wobbleGoal {...}
        extendWobbleGoal = gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.x);

        if (extendWobbleGoal) {
            if (gamepadActions.isToggled(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_right)) {
                wobbleServo.setPosition(1);
            } else {
                wobbleServo.setPosition(0);
            }
        } else {
            gamepadActions.setToggleStateFor(false, GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.bumper_right);
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
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("BL", backLeft.getCurrentPosition());
        telemetry.addData("BR", backRight.getCurrentPosition());
    }
}
