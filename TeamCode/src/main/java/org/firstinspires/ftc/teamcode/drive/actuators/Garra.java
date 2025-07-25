package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Garra extends OpMode {
    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo garrinha;
    public void init() {
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");
        garrinha = hardwareMap.get(Servo.class, "garrinha");
    }

    public void loop() {

        if (gamepad1.right_bumper){
            rotate.setPosition(0);
        }
        if (gamepad1.left_bumper){
            rotate.setPosition(1);
        }
        if (gamepad1.dpad_right){
            rotate.setPosition(0.5);
        }


            if (gamepad1.a) {
                garra.setPosition(0.3);
            }
            if (gamepad1.b) {
                garra.setPosition(0.7);
            }
            if (gamepad1.x) {
                pleft.setPosition(0.75);
                pright.setPosition(0.25);
            }
            if (gamepad1.y) {
                pleft.setPosition(0);
                pright.setPosition(1);
            }
            if (gamepad2.a){
                garrinha.setPosition(0);
            }
            if (gamepad2.b){
                garrinha.setPosition(1);
            }
        }
    }