package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.routines.Routine;

public class ClawSystem extends Subsystem
{
    private Servo shoulder_servo;
    private Servo claw_servo_l;
    private Servo claw_servo_r;


    public Servo getShoulder_servo() {
        return shoulder_servo;
    }

    public Servo getClaw_servo_l() {
        return claw_servo_l;
    }

    public Servo getClaw_servo_r() {
        return claw_servo_r;
    }

    final double OPEN = 0.25;
    final double CLOSE = 0.75;// TEST THEM OUT AFTERNOON

    private boolean openChanged = false;

    private  boolean isOpen = true;
    public void setClawPos(double pos){
        this.claw_servo_l.setPosition(pos);
        this.claw_servo_r.setPosition(pos);
    }

    public void openClaw(){
        this.claw_servo_l.setPosition(this.OPEN);
        this.claw_servo_r.setPosition(this.OPEN);
    }

    public void closeClaw(){
        this.claw_servo_l.setPosition(this.CLOSE);
        this.claw_servo_r.setPosition(this.CLOSE);
    }

    public void toggleClaw(boolean toggle){

        if(toggle && !openChanged){
            if(isOpen){
                closeClaw();
                isOpen = false;
            }

            else {
                openClaw();
                isOpen = true;
            }
            openChanged = true;
        }
        else if (!toggle){
            openChanged = false;
        }

    }

    public ClawSystem(Routine routine) {
        super(routine);
    }
}