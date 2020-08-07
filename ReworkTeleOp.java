package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ReworkTeleOp extends LinearOpMode {
    ReworkRobot robot;

    public void runOpMode() {
        initRobot();

        waitForStart();

        while (opModeIsActive()) {
            drive();
        }
    }

    private void initRobot() {
        robot = new ReworkRobot(hardwareMap);
    }

    private void drive() {
        robot.drivetrain.setStates(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }
}
