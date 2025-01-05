package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Moving {


    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Gamepad gamepad;

    public void setHW(HardwareMap hwm, Telemetry tm, Gamepad gp) {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hwm.dcMotor.get("frontLeft");
        backLeftMotor = hwm.dcMotor.get("backLeft");
        frontRightMotor = hwm.dcMotor.get("frontRight");
        backRightMotor = hwm.dcMotor.get("backRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = gp;
    }

    public void move() {

        double multiplier = 0.45;
        if (gamepad.left_bumper) { multiplier = 0.9; }
        if(gamepad.right_bumper) { multiplier = 0.25;}

        double y        = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x        = gamepad.left_stick_x * 1; // Counteract imperfect strafing
        double rotation = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x + rotation) / denominator * multiplier*1.1;
        double backLeftPower = (y - x + rotation) / denominator * multiplier*1.1;
        double frontRightPower = (y - x - rotation) / denominator * multiplier;
        double backRightPower = (y + x - rotation) / denominator * multiplier;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }


}