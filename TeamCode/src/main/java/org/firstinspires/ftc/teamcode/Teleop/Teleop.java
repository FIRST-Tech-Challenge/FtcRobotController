package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        // Servo servo = hardwareMap.servo.get("servo1");

        Robot robot = new Robot(this, timer);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) { // clearer nomenclature for variables
            double ly = Math.pow(gamepad1.left_stick_y, 3);
            double lx = Math.pow(gamepad1.left_stick_x, 3);
            double rx = Math.pow(gamepad1.right_stick_x, 3);
            double ry = Math.pow(gamepad1.right_stick_y, 3);

            double r = Math.hypot(lx, ly);
            double robotAngle = Math.atan2(ly, lx) - Math.PI / 4;
            double rearLeftPower = r * Math.sin(robotAngle) + (rx / 2);
            double frontLeftPower = r * Math.cos(robotAngle) + (rx / 2);
            double rearRightPower = r * Math.cos(robotAngle) - (rx / 2);
            double frontRightPower = r * Math.sin(robotAngle) - (rx / 2);
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(rearLeftPower);
            backRight.setPower(rearRightPower);
        }
    }
}