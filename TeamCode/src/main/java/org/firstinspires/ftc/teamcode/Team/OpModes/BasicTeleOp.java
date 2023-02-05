package org.firstinspires.ftc.teamcode.Team.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic Testing Mecanum TeleOp")

public class BasicTeleOp extends LinearOpMode {
    public Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        //drive controls
        double accel;
        double rotate;
        double powR;
        double powL;

        while (opModeIsActive() && Test.truthy()) {
            accel = gamepad1.left_stick_y;
            rotate = gamepad1.left_stick_x;
            //Determines ratio of motor powers (by sides) using the right stick
            double rightRatio = 0.5 - (0.5 * rotate);
            double leftRatio = 0.5 + (0.5 * rotate);
            //Declares the maximum power any side can have
            double maxRatio = 1;
            //If we're turning left, the right motor should be at maximum power, so it decides the maxRatio. If we're turning right, vice versa.
            if (rotate < 0) {
                maxRatio = 1 / rightRatio;
            } else {
                maxRatio = 1 / leftRatio;
            }
            //Uses maxRatio to max out the motor ratio so that one side is always at full power.
            powR = rightRatio * maxRatio;
            powL = leftRatio * maxRatio;
            //Uses left trigger to determine slowdown.

            // variables used in lambda functions *MUST* be marked as final
            final double powRight = -powR * accel;
            final double powLeft = -powL * accel;

            if (gamepad1.left_bumper | gamepad1.right_bumper) {
                robot.pivotTurn(1, gamepad1.left_bumper, gamepad1.right_bumper);
            } else if (Math.abs(gamepad1.right_stick_x) > 0.5 | Math.abs(gamepad1.right_stick_y) > 0.5) {
                robot.octoStrafe(-0.7, gamepad1.right_stick_y > 0.5, gamepad1.right_stick_y < -0.5, gamepad1.right_stick_x > 0.5, gamepad1.right_stick_x < -0.5);
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                robot.mapRight(m -> m.setPower(powRight));
                robot.mapLeft(m -> m.setPower(powLeft));
            } else {
                robot.stop();
            }
            telemetry.update();
        }
    }
}