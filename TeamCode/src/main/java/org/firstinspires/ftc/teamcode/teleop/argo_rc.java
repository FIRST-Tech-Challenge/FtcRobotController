package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpWord_RC")
public class argo_rc extends argonautRobot {

    @Override
    public void drive(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor,
                      double x, double y, double rx, double botHeading) {
        //*************************
        //* Robot-centric driving *
        //*************************
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if(gamepad1.b) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        } else {
            //precision mode half power drive
            frontLeftMotor.setPower(0.5 * frontLeftPower);
            backLeftMotor.setPower(0.5 * backLeftPower);
            frontRightMotor.setPower(0.5 * frontRightPower);
            backRightMotor.setPower(0.5 * backRightPower);
        }
    }

}
