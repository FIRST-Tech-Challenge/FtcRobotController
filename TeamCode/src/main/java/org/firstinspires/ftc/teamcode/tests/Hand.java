package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Hand extends LinearOpMode {
    private Servo Hand_rotator;
    private Servo Hand_Grip;
    @Override

    public void runOpMode() throws InterruptedException {

    }
    public Hand(String rotator_name, String hand_name){
        this.Hand_Grip = hardwareMap.get(Servo.class, hand_name);
        this.Hand_rotator = hardwareMap.get(Servo.class, rotator_name);
    }

    public void rotate(){
        this.Hand_rotator.setPosition(
                this.Hand_rotator.getPosition() + (gamepad2.a ? 0.1 : (gamepad2.b ? -0.1: 0))
        );
    }
}
