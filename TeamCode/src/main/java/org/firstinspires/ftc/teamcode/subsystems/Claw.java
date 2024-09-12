package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;
    private double openPos, closePos;

    private boolean isOpen;

    public Claw(HardwareMap hw){
        this(hw,"Claw");
    }

    public Claw(HardwareMap hw, String servoName){
        claw = hw.get(Servo.class,servoName);
        openPos = 1;
        closePos = 0;
        isOpen = false;
    }

    public void setOpenPos(double openPos) {
        this.openPos = openPos;
    }

    public void setClosePos(double closePos) {
        this.closePos = closePos;
    }

    public void openClaw(){
        claw.setPosition(openPos);
    }

    public void closeClaw(){
        claw.setPosition(closePos);
    }

    public void toggleClaw(){
        if (isOpen) {
            closeClaw();
        }
        else{
            openClaw();
        }
    }
}
