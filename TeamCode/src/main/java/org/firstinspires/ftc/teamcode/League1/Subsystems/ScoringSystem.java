package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;

public class ScoringSystem {

    public enum ExtensionHeight{
        HIGH,
        MEDIUM
    }

    //TODO: See if this is what we are actually using
    DcMotorEx rLift, lLift;
    Servo grabber, /*lLinkage,*/ Linkage; //dont know whether there are two linkage servos or one
    private Robot robot;

    Constants constants;
    boolean fullyExtended = false;

    public ScoringSystem(HardwareMap hardwareMap, Robot robot, Constants constants){
        this.robot = robot;
        this.constants = constants;
        rLift = hardwareMap.get(DcMotorEx.class, "RightLift");
        lLift = hardwareMap.get(DcMotorEx.class, "LeftLift");

        //lLinkage = hardwareMap.get(Servo.class, "LeftLinkage");
        Linkage = hardwareMap.get(Servo.class, "RightLinkage");

        grabber = hardwareMap.get(Servo.class, "Grabber");
    }

    //TODO: change to velocity later if possible and add PID implementation
    public void moveToPosition(int tics, double power){
        if(!fullyExtended || tics == 0){

        }



    }

    public void extend(ExtensionHeight height){
        if(height == ExtensionHeight.HIGH){
            moveToPosition(constants.highHeight, 0.2);
        }else{
            moveToPosition(constants.mediumHeight, 0.2);
        }

    }

    private void setPower(double power){
        rLift.setPower(power);
        lLift.setPower(power);
    }

    private void setVelocity(int velocity){
        rLift.setVelocity(velocity);
        lLift.setVelocity(velocity);
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
