package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecimanGrabber {

    private Servo grabber = null;


    public void Init(HardwareMap hardwareMap) {
        grabber = hardwareMap.get(Servo.class, "specimangrabber");
    }

    public void Open() {
        grabber.setPosition(1.0);
    }

    public void Close() {
        grabber.setPosition(0.1);
    }

}
