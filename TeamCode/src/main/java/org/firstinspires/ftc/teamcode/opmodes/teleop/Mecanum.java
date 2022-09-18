package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

//  Controls:
// left stick forward and backward
// right stick left and right to strafe
// left stick left and right to turn

@TeleOp(name = "Mecanum")
public class Mecanum extends LinearOpMode {

    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive;
    double driveSpeed = 1.0;
    @Override
    public void runOpMode() {
        TurtleRobot robot = new TurtleRobot(this);
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            
            frontRightDrive = (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x)*driveSpeed;
            frontLeftDrive  = (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x)*driveSpeed;
            backRightDrive  = (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x)*driveSpeed;
            backLeftDrive   = (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x)*driveSpeed;

            robot.rightfrontmotor.setPower(frontRightDrive);
            robot.rightbackmotor.setPower(backRightDrive);
            robot.leftbackmotor.setPower(backLeftDrive);
            robot.leftfrontmotor.setPower(frontLeftDrive);

            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive speed = ", robot.rightfrontmotor);
            telemetry.addData("Front left drive speed  = ", robot.leftfrontmotor);
            telemetry.addData("Back right drive speed  = ", robot.rightbackmotor);
            telemetry.addData("Back left drive speed   = ", robot.leftbackmotor);
            telemetry.update();
            }
        }
    }
}