package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public class Grabber {
    public CRServo Servo1;
    public CRServo Servo2;
    public void stop() {
        Servo1.setPower(0);
        Servo2.setPower(0);

    }
    public void grabber1(float power){
        Servo1.setPower(power);
    }
    public void grabber2(float power){
        Servo2.setPower(power);
    }

}
