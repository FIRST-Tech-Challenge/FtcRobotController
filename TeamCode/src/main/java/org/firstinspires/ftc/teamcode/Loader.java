package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Loader {
    private DcMotor motor;
    public Loader(DcMotor motor) {
        this.motor = motor;
    }
    public void on() {
        motor.setPower(1.0);
    }
    public void off() {
        motor.setPower(0.0);
    }
}
