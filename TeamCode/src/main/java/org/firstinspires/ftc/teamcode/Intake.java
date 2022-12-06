package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo armLeft;
    private Servo armRight;
    private Servo wrist;
    private Servo claw;

    double armMax = 1;
    double armMin = 0;

    double retracted = .9;
    double extended = armMin;
    double armMid = .50;
    double sizing = .65;

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
    public enum Height{EXTENDED, UPRIGHT, RETRACTED, SIZING}

    //moves arm to a position
    public void runArm(double pos) throws InterruptedException{
        if (pos>armMax){
            newPos = armMax;
        }
        else if (pos<armMin){
            newPos = armMin;
        }
        else {
            newPos = pos;
        }

        armLeft.setPosition(newPos);
        armRight.setPosition(1-newPos);

        //if matched, make the wrist keep the cone upright
        if (wristMode== WristMode.MATCHED){
            /*if(newPos>armMid&&claw.getPosition()>0.4) {
                runWrist(1);
            }
            else if(newPos>.5){
                runWrist(newPos + endMod);
            }*/
            if (Math.abs(newPos-getArmPos())>.5){
                //sleep(200);
            }
            runWrist(newPos);
        }
    }

    //moves arm to a position, changing the wristmode during only this function
    public void runArm(double pos, WristMode newWristMode){
        if (pos>armMax){
            newPos = armMax;
        }
        else if (pos>armMin){
            newPos = armMin;
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
        else if (pos>armMin){
            newPos = armMin;
        }
        else if (pos>armMin){
            newPos = armMin;
        }
        else {
            newPos = pos;
        }

        armLeft.setPosition(newPos);
        armRight.setPosition(1-newPos);

        //make the wrist keep the cone upright, with the modification

        runWrist(newPos+wristMod);

    }

    public double runArm(Height height) throws InterruptedException{
        switch(height){
            case EXTENDED:
                runArm(extended);
                return extended;
            case UPRIGHT:
                runArm(armMid);
                return armMid;
            case RETRACTED:
                runArm(retracted);
                return retracted;
            case SIZING:
                runArm(sizing);
                return sizing;
            default:
                return armMid;
        }
    }

    //this method moves the wrist to a position
    public void runWrist(double pos){
        wrist.setPosition(pos);
    }

    //this method changes whether the wrist matches rotation of the arm
    public void changeWristMode(WristMode newWristMode){
        wristMode = newWristMode;
    }

    //this method moves the claw to a position
    public void runClaw(double pos){
        claw.setPosition(pos);
    }

    public void openClaw(){
        runClaw(.05);
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

    public void intakeCone() throws InterruptedException{
        closeClaw();
        sleep(25);
        runArm(Height.RETRACTED);
        sleep(200);
        openClaw();
        sleep(50);
        runArm(Height.UPRIGHT);
    }

    //this method returns the arm's position
    //0 is all the way out, 1 is all the way in
    public double getArmPos(){
        return armLeft.getPosition();
    }
}
