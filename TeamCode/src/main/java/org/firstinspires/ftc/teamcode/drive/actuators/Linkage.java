package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//@TeleOp
public class Linkage extends OpMode {
    Servo lright;
    Servo lleft;
    public void init(){
        lright = hardwareMap.get(Servo.class,"lright");
        lleft = hardwareMap.get(Servo.class,"lleft");
    }
    public void loop(){
        if (gamepad1.a){
            lright.setPosition(1);
            lleft.setPosition(0);
        }
        if (gamepad1.b){
            lright.setPosition(0.6);
            lleft.setPosition(0.7);
        }
    }
}
