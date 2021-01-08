package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;

public class WobbleSubsystem extends ServoSubsystem {
    public Servo servo1;
    public Servo servo2;

    public enum ArmPosition{
        RAISED(1), LOWERED(0);
        public double position;
        ArmPosition(double pos) {
            position = pos;
        }
        public double getPosition(){
            return position;
        }
    }

    public enum ClawPosition{
        OPEN(1), CLOSED(0);
        public double position;
        ClawPosition(double pos) {
            position = pos;
        }
        public double getPosition(){
            return position;
        }
    }

    public ArmPosition armPosition;
    public ClawPosition clawPosition;
    //(a+b)/2=armpos    (a-b+1)/2=clawpos
    //
    public WobbleSubsystem(Servo s1, Servo s2){
        servo1 = s1;
        servo2 = s2;
        armPosition = ArmPosition.LOWERED;
        clawPosition = ClawPosition.CLOSED;
    }

    public void setArmPosition(ArmPosition pos){

    }


}
