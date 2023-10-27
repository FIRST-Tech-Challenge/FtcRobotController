package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Fourbar {
    private final Servo fourbar;

    //values need to be changed
    public static double outtake = 1;

    //approximating angle as 180 degrees
    public static double storage = 0;
    public static boolean isOuttakePosition;


    public Fourbar(OpMode opMode) {
        fourbar = opMode.hardwareMap.servo.get("fourBarServo");
        fourbar.setDirection(Servo.Direction.FORWARD);
        isOuttakePosition= false;
    }

    public void outtake(){
        fourbar.setPosition(outtake);
        isOuttakePosition = true;
    }

    public void storage(){
        fourbar.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(storage);
        isOuttakePosition = false;
    }

    public void runManualOuttake(double position){
        fourbar.setPosition(position);
    }

    public void runManualStorage(double position){
        fourbar.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(position);
    }

    public boolean getIsOuttakePos(){
        return isOuttakePosition;
    }

}
