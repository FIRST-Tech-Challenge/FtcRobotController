package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//brendan
public class GrabberClass {

    public Servo grabber;

    String servoName;

    HardwareMap hardwareMap;

    public GrabberClass(String Servo) {

        servoName = Servo;


    }

    public void initialize() {

        grabber = hardwareMap.get(Servo.class, servoName);

    }

    public void grabberMove(double position) {

        grabber.setPosition(position);


    }

}
