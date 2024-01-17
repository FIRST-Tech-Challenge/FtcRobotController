package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AlderMotor {

    private DcMotor motor;

    public AlderMotor(DcMotor m) {
        motor = m;
    }

    public void run(double par) {
        motor.setPower(par);
    }

}
