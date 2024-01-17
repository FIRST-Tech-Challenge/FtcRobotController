package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Basic_Drive_Files.RobotCentricMecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotCentric {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    public RobotCentric(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
        fl = m1;
        fr = m2;
        bl = m3;
        br = m4;
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx) {
        //Robot-Centric Mecanum
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flpwr = (y + x + rx) / denominator;
        double blpwr = (y - x + rx) / denominator;
        double frpwr = (y - x - rx) / denominator;
        double brpwr = (y + x - rx) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }
}
