package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//@TeleOp
public class Bucket extends OpMode {
    Servo right;
    Servo left;
    public void init(){
        right = hardwareMap.get(Servo.class,"bright");
        left = hardwareMap.get(Servo.class,"bleft");
    }
    public void loop(){
        if (gamepad1.a) {
            right.setPosition(1);
            left.setPosition(0);
        }
        if (gamepad1.b){
            right.setPosition(0.2);
            left.setPosition(0.8);
        }
    }


}
