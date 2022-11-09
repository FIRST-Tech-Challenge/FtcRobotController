package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo armLeft;
    private Servo armRight;
    private Servo wrist;
    private Servo claw;

    double armMax = .8;
    double armMin = 0;
    double armMid = .40;
    double endMod = 0;
    double newPos;

    WristMode wristMode;

    //set up servos and variables
    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        wristMode = WristMode.MATCHED;
    }

    public enum WristMode{INDEPENDENT, MATCHED, SIDEWAYS}
    public enum Height{EXTENDED, UPRIGHT, RETRACTED}

    //moves arm to a position
    public void runArm(double pos){
        if (pos>armMax){
            newPos = armMax;
        }
        else {
            newPos = pos;
        }

        armLeft.setPosition(newPos);
        armRight.setPosition(1-newPos);

        //if matched, make the wrist keep the cone upright
        if (wristMode== WristMode.MATCHED){
            if(newPos>.7){
                runWrist(newPos+endMod);
            }
            else {
                runWrist(newPos);
            }
        }
    }

    //moves arm to a position, changing the wristmode during only this function
    public void runArm(double pos, WristMode newWristMode){
        if (pos>armMax){
            newPos = armMax;
        }
        else {
            newPos = pos;
        }

        armLeft.setPosition(newPos);
        armRight.setPosition(1-newPos);

        //if matched, make the wrist keep the cone upright
        if (newWristMode== WristMode.MATCHED){
            if(newPos>.7){
                runWrist(newPos+endMod);
            }
            else {
                runWrist(newPos);
            }
        }
    }

    //moves arm to a position
    public void runArm(double pos, double wristMod){
        if (pos>armMax){
            newPos = armMax;
        }
        else {
            newPos = pos;
        }

        armLeft.setPosition(newPos);
        armRight.setPosition(1-newPos);

        //make the wrist keep the cone upright, with the modification

        runWrist(newPos+wristMod);

    }

    public double runArm(Height height){
        switch(height){
            case EXTENDED:
                runArm(armMin);
                return armMin;
            case UPRIGHT:
                runArm(armMid);
                return armMid;
            case RETRACTED:
                runArm(armMax);
                return armMax;
            default:
                return armMid;
        }
    }

    //this method moves the wrist to a position
    public void runWrist(double pos){
        wrist.setPosition(pos);
    }

    //this method moves the claw to a position
    public void runClaw(double pos){
        claw.setPosition(pos);
    }

    //this method changes whether the wrist matches rotation of the arm
    public void changeWristMode(WristMode newWristMode){
        wristMode = newWristMode;
    }

    public void openClaw(){
        runClaw(.2);
    }
    public void closeClaw(){
        runClaw(.55);
    }

    public void toggleClaw(){
        if (claw.getPosition()>.3){
            openClaw();
        }
        else{closeClaw();}
    }

    //this method returns the arm's position
    //0 is all the way out, 1 is all the way in
    public double getArmPos(){
        return armLeft.getPosition();
    }
}
