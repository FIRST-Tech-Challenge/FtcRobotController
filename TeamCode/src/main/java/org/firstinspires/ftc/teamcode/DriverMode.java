package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(this);
    }

    @Override
    public void loop() {
        /*
         * Drive robot based on joystick input from gamepad1
         * Right stick moves the robot forwards, backwards and sideways.
         * Left stick rotates it.
         */
        hardware.getWheels().drive(
                gamepad1.right_stick_x,
                gamepad1.right_stick_y,
                gamepad1.left_stick_x
        );

        /*
         * The right joystick on gamepad2 controls both arm rotation and extension:
         * Up rotates the arm up, and down rotates it down.
         * Right extends the arm, left retracts it.
         */
        hardware.getArm().rotateArm(gamepad2.right_stick_y);
        hardware.getArm().extendArm(gamepad2.right_stick_x);

        /*
         * The left joystick on gamepad2 controls the claw rotation:
         * Right rotates the claw clockwise on the X-axis, left rotates it counterclockwise.
         * Up rotates the claw clockwise on the Z-axis, left rotates it counterclockwise.
         */
        hardware.getArm().rotateClawXServo(gamepad2.left_stick_x);
        hardware.getArm().rotateClawZServo(gamepad2.left_stick_y);

        if (gamepad2.a) {
            hardware.getArm().startIntake();
        } else if (gamepad2.b) {
            hardware.getArm().stopIntake();
        } else if (gamepad2.y) {
            hardware.getArm().ejectIntake();
        }

        // Update the information from the robot
        telemetry.update();
    }
}