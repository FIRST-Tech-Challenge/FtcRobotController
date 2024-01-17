package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AlderMecanum {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    public AlderMecanum(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
        fl = m1;
        fr = m2;
        bl = m3;
        br = m4;
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower); //Wrong Variables were inside power function, power also has to be capital in .setPower();
        fr.setPower(backLeftPower); //Wrong Variables were inside power function, power also has to be capital in .setPower();
        bl.setPower(frontRightPower); //Wrong Variables were inside power function, power also has to be capital in .setPower();
        br.setPower(backRightPower); //Wrong Variables were inside power function, power also has to be capital in .setPower();

    }
    
}