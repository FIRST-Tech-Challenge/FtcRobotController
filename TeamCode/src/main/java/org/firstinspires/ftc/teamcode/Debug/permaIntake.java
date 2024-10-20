package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class permaIntake extends LinearOpMode {

    Servo intakeL, intakeR;

    public void runOpMode(){

        initRobot();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                intakeL.setPosition(1);
                intakeR.setPosition(1);
            }
            else{
                intakeL.setPosition(0);
                intakeR.setPosition(0);
            }
        }

    }

    public void initRobot(){
        intakeL = hardwareMap.get(Servo.class,"left");
        intakeR = hardwareMap.get(Servo.class,"right");

        intakeL.setDirection(Servo.Direction.REVERSE);
        intakeR.setDirection(Servo.Direction.REVERSE);
    }
}
