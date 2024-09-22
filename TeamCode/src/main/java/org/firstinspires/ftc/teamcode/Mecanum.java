package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class Mecanum {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    public static Mecanum Init(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft) {
        Mecanum m = new Mecanum();
        m.fl = frontLeft;
        m.fr = frontRight;
        m.br = backRight;
        m.bl = backLeft;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        m.fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m.br.setDirection(DcMotorSimple.Direction.REVERSE);
        return m;
    }


    public void Move(double y, double x, double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }
}
