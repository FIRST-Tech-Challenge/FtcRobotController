package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Loader {
    private DcMotor motor;
    public Loader(DcMotor motor) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0.0);
    }
    public void on() {
        motor.setPower(1.0);
    }
    public void off() {
        motor.setPower(0.0);
    }
}
