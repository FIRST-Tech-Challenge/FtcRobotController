package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// initializes the grabber for the robot
public class GrabberClass {

    // assuming the grabber has a servo
    public Servo grabber;

    String servoName;

    HardwareMap hardwareMap;

    // constructor
    public GrabberClass(String Servo) {

        servoName = Servo;


    }

    // basic functions
    public void initialize() {

        grabber = hardwareMap.get(Servo.class, servoName);

    }

    public void grabberMove(double position) {

        grabber.setPosition(position);


    }

}
