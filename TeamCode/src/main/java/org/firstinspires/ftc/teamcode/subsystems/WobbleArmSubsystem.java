package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;

public class WobbleArmSubsystem extends ServoSubsystem {
    public Servo armServo;
    public enum ArmPosition{
        RAISED(1), LOWERED(0);
        double position;
        ArmPosition(double val){
            position = val;
        }
        public double getPosition(){
            return position;
        }
    }
    public ArmPosition position;
    public WobbleArmSubsystem(Servo s){
        super(s);
        armServo = s;
        position = ArmPosition.LOWERED;
    }
    public void raise(){
        setPosition(ArmPosition.RAISED.getPosition());
        position = ArmPosition.RAISED;
    }

    public void lower(){
        setPosition(ArmPosition.LOWERED.getPosition());
        position = ArmPosition.LOWERED;
    }

    public ArmPosition getArmPosition(){
        return position;
    }

}
