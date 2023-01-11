package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team.ComplexRobots.Robot;

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

        while (opModeIsActive()) {
            accel = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
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
            robot.RFMotor.setPower(-powR * accel);
            robot.RBMotor.setPower(-powR * accel);
            robot.LFMotor.setPower(-powL * accel);
            robot.LBMotor.setPower(-powL * accel);

            robot.pivotTurn(1, gamepad1.left_bumper, gamepad1.right_bumper);
            //Strafing controls
            robot.octoStrafe(1, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
            telemetry.update();
        }
    }
}