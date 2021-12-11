package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    public Outtake (HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motorOuttake");
        servo1 = hardwareMap.servo.get("servoOuttake");
        motor1.setPower(0.3);
    }


    public void FlapOpen() {
        servo1.setPosition(0.2);
    }

    public void FlapClosed() {
        servo1.setPosition(0.0);
    }

    private DcMotor motor1;
    private Servo servo1;


}
