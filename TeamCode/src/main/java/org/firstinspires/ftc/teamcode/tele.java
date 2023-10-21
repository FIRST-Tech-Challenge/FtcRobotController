package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Tele", group = "Pushbot")
public class tele extends OpMode {
    Hardware robot = new Hardware();

    public void init() {

        robot.init(hardwareMap);

    }

    public void start() {

    }

    public void loop() {

        double speedLimit = 80;
        double speedLimitValue = speedLimit/100;

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.leftDrive.setPower(frontLeftPower * speedLimitValue);
        robot.leftBackDrive.setPower(backLeftPower * speedLimitValue);
        robot.rightDrive.setPower(frontRightPower * speedLimitValue);
        robot.rightBackDrive.setPower(backRightPower * speedLimitValue);

    }

}
