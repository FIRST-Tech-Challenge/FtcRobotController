package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private Servo servo;
    private ElapsedTime timer;
    private DcMotor motor;

    public Launcher(Servo servo, DcMotor motor ) {
        this.servo = servo;
        this.motor = motor;
    }

    public void launch() {
        motor.setPower(1.0);
        servo.setPosition(1);
        timer.reset();
        while(timer.milliseconds() < 2000){}
        servo.setPosition(0);
        motor.setPower(0.0);
    }

}
