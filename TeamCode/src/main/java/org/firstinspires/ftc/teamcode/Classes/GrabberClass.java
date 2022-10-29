package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

<<<<<<< Updated upstream
// initializes the grabber for the robot
public class GrabberClass {
=======
public class GrabberClass extends LinearSlideClass {
>>>>>>> Stashed changes

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
        initializeLinearSlide();

    }

    public void grabberMove(double position) {

        grabber.setPosition(position);



    }

    public void grabAndGoUp(double position, double power, int time) throws InterruptedException {

        grabber.setPosition(position);

        linearSlideUp(power, time);


    }


    public void grabAndGoDown(double position, double power, int time) throws InterruptedException {

        grabber.setPosition(position);

        linearSlideDown(power, time);


    }

}
