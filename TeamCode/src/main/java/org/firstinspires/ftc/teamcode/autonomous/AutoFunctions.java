package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.commons.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoFunctions {

    private final RobotHardware robot;
    private final LinearOpMode opMode; // Store the LinearOpMode instance

    // Constructor that takes RobotHardware and LinearOpMode as parameters
    public AutoFunctions(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    // Method to set power to all four drive motors for a specified time using LinearOpMode's sleep()
    public void basePower(double power, int time) {
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);
        opMode.sleep(time);
        // Stop all motors
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        opMode.sleep(1000);
    }
    public void clawPower (double power, int time) {
        robot.leftClaw.setPower(power);
        robot.rightClaw.setPower(power);
        opMode.sleep(time);
        robot.leftClaw.setPower(0);
        robot.rightClaw.setPower(0);
        opMode.sleep(1000);
    }
    public void hangPower(double power, int time) {
        robot.leftHang.setPower(power);
        robot.rightHang.setPower(power);
        opMode.sleep(time);
        robot.leftHang.setPower(0);
        robot.rightHang.setPower(0);
        opMode.sleep(1000);
    }
}