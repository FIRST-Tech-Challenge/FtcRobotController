package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket extends OpMode {
    Servo right;
    Servo left;
    public void init(){
        right = hardwareMap.get(Servo.class,"dir");
        left = hardwareMap.get(Servo.class,"esq");
    }
    public void loop(){
        if (gamepad1.a) {
            right.setPosition(1);
            left.setPosition(0);
        }
        if (gamepad1.b){
            right.setPosition(0.6);
            left.setPosition(0.4);
        }
    }
}
