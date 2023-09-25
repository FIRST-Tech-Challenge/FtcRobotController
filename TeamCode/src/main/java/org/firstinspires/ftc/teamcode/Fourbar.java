package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Fourbar {
    private final Servo fourbarOne;
    private final Servo fourbarTwo;

    //values need to be changed
    public static double outtake = 0.6;
    public static double storage = 0.6;
    public static boolean isOuttakePosition;


    public Fourbar(OpMode opMode) {
        fourbarOne = opMode.hardwareMap.servo.get("fourbarOne");
        fourbarOne.setDirection(Servo.Direction.FORWARD);
        fourbarTwo = opMode.hardwareMap.servo.get("fourbarTwo");
        fourbarTwo.setDirection(Servo.Direction.FORWARD);
        isOuttakePosition= false;
    }

    public void Outtake(){
        fourbarOne.setPosition(outtake);
        fourbarTwo.setPosition(outtake);
        isOuttakePosition= true;
    }

    public void Storage(){
        fourbarOne.setDirection(Servo.Direction.REVERSE);
        fourbarTwo.setDirection(Servo.Direction.REVERSE);
        fourbarOne.setPosition(storage);
        fourbarTwo.setPosition(storage);
        isOuttakePosition= false;
    }

    public void runManualOuttake(double position){
        fourbarOne.setPosition(position);
        fourbarTwo.setPosition(position);
    }

    public void runManualStorage(double position){
        fourbarOne.setDirection(Servo.Direction.REVERSE);
        fourbarTwo.setDirection(Servo.Direction.REVERSE);
        fourbarOne.setPosition(position);
        fourbarTwo.setPosition(position);
    }

}
