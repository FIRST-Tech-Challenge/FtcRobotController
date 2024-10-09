package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class TeleOpMode extends LinearOpMode {
    DcMotorEx topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

    double maxSpeed;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            robot.teleOpDrive(drive,strafe,rotate);
        }
    }
}
