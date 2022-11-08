package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw{
    private Servo wristJoint,clawJoint;
    private boolean isOpen;
    private final double OPEN = 0.45;
    private final double CLOSE = 0.15;
    private final double FACINGFORWARD = 0;
    private final double FACINGBACKWARD = 1;

    public Claw(HardwareMap hardwareMap){
        wristJoint = hardwareMap.get(Servo.class,"WRIST");
        clawJoint = hardwareMap.get(Servo.class, "CLAW");

        isOpen = true;
    }

    public void openOrCloseClaw(){
        if(isOpen){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
        isOpen = !isOpen;
    }

    public void rotateWristIfForward(){
        wristJoint.setPosition(FACINGFORWARD);
    }

    public void rotateWristIfBackward(){
        wristJoint.setPosition(FACINGBACKWARD);
    }
}
