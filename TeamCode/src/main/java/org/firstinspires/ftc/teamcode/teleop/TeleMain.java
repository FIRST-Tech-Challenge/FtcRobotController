package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helpers.DriveTrain;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;

public class TeleMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        DriveTrain dt = new DriveTrain(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
        waitForStart();
        while (opModeIsActive()) {
            dt.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
