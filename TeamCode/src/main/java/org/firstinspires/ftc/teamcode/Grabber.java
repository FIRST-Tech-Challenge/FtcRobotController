package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.robotcore.hardware.Servo;
public class Grabber {
    public Servo Servo1;
    public Servo Servo2;
    public void stop() {
        Servo1.setPosition(0);
        Servo2.setPosition(0);

    }
    public void grabber1(float position){
        Servo1.setPosition(position);
    }
    public void grabber2(float position){
        Servo2.setPosition(position);
    }

}
