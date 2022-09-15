package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringSystem {

    //TODO: See if this is what we are actually using
    DcMotorEx rLift, lLift;
    Servo grabber, lLinkage, rLinkage; //dont know whether there r two linkage servos or one

    public ScoringSystem(HardwareMap hardwareMap){
        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        lLinkage = hardwareMap.get(Servo.class, "LeftLinkage");
        rLinkage = hardwareMap.get(Servo.class, "RightLinkage");

        grabber = hardwareMap.get(Servo.class, "Grabber");
    }

    //TODO: change to velocity later if possible and add PID implementation
    public void moveToPosition(int tics, double power){

    }

    public void setPower(double power){
        rLift.setPower(power);
        lLift.setPower(power);
    }

    public void setLinkagePosition(double position){
        //TODO: tune position values
        /*lLinkage.setPosition();
        rLinkage.setPosition();*/
    }

    public void setGrabberPosition(double position){
        grabber.setPosition(position);
    }
}
