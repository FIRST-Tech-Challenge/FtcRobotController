package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class IntakeClaw extends SubsystemBase {

    public enum ClawState{
        OPEN(1),
        CLOSE(0),
        READY(0.5),
        COLLECT(0.6),
        TRANSITION(0.4);


    public double pos;

    private ClawState(double pos){this.pos = pos;}
}
    private Servo intakeClaw;
    private Servo clawFlip;
    private Servo intakePivot;

    public IntakeClaw(HardwareMap hMap){
        this.intakeClaw = hMap.get(Servo.class, "intakeClaw");


        this.clawFlip = hMap.get(Servo.class, "clawFlip");
    }


    public Command openClaw(){
        return new InstantCommand(()->intakeClaw.setPosition(ClawState.OPEN.pos));
    }


    public Command closeClaw(){
        return new InstantCommand(()-> intakeClaw.setPosition(ClawState.CLOSE.pos));
    }

    public Command rotateto90(){
        return new InstantCommand(()-> clawFlip.setPosition(0.4));
    }

    public Command rotateTo0(){
        return new InstantCommand(()->clawFlip.setPosition(0));
    }

    public Command pivotToReady(){
        return new InstantCommand(()-> intakePivot.setPosition(ClawState.READY.pos));
    }

    public Command pivotToCollect(){
        return new InstantCommand(()-> intakePivot.setPosition(ClawState.COLLECT.pos));
    }

    public Command pivotToTransition(){
        return new InstantCommand(()-> intakePivot.setPosition(ClawState.TRANSITION.pos));
    }

}

