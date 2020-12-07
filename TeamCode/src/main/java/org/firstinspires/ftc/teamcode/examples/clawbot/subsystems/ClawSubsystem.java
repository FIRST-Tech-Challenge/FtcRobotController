package org.firstinspires.ftc.teamcode.examples.clawbot.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;

public class ClawSubsystem extends ServoSubsystem {
    //subsystem enums
    public enum ClawPosition{
        OPEN(0), CLOSED(1);
        double position;
        ClawPosition(double pos){
            position = pos;
        }
        public double getPosition(){
            return position;
        }
        public ClawPosition invert(){
            return this == OPEN ? CLOSED : OPEN;
        }
    }
    public ClawPosition position;
    public ClawSubsystem(Servo s){
        super(s);
        position = ClawPosition.OPEN;
    }
    public void setClawPosition(ClawPosition pos){
        setPosition(pos.getPosition());
        position = pos;
    }
}
