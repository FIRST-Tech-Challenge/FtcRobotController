package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum_Driving {

    DcMotor fl, fr, rl, rr;

    public Mecanum_Driving(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void drive(double y, double x, double rx) {

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double rlPower = y - x + rx;
        double rrPower = y + x - rx;

        double max = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(rlPower), Math.abs(rrPower))
        );

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            rlPower /= max;
            rrPower /= max;
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        rl.setPower(rlPower);
        rr.setPower(rrPower);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}