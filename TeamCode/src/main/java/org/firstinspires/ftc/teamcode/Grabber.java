package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;
public class Grabber {
    public Servo Servo1;
    public Servo Servo2;
    public void open() {
        Servo1.setPosition(0);
        Servo2.setPosition(0);

    }
    public void Close() {
        Servo1.setPosition(90);
        Servo2.setPosition(90);
    }
}
