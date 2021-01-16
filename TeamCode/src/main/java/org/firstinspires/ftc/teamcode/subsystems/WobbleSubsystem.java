package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.motor.MotorSubsystem;
import com.technototes.library.subsystem.servo.ServoSubsystem;
import com.technototes.logger.Stated;

public class WobbleSubsystem extends ServoSubsystem implements Stated<String> {
    public Servo servo1;
    public Servo servo2;

    public enum ArmPosition{
        RAISED(1), LOWERED(-1);
        public double position;
        ArmPosition(double pos) {
            position = pos;
        }
        public double getPosition(){
            return position;
        }
    }

    public enum ClawPosition{
        OPEN(1), CLOSED(-1);
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
        super(s1, s2);
        servo1 = s1;
        servo2 = s2;
        armPosition = ArmPosition.LOWERED;
        clawPosition = ClawPosition.CLOSED;
    }




    public void setClawPosition(ClawPosition pos){
        if(clawPosition != pos) {
            servo1.setPosition(servo1.getPosition() - pos.getPosition());
            //TODO servo1.addToPosition(-pos.getPosition());
            servo2.setPosition(servo2.getPosition() + pos.getPosition());
            clawPosition = pos;
        }

    }
    public void setArmPosition(ArmPosition pos){
        if(armPosition != pos) {
            servo1.setPosition(servo1.getPosition() + pos.getPosition());
            servo2.setPosition(servo2.getPosition() + pos.getPosition());
            armPosition = pos;
        }
    }

    @Override
    public String getState() {
        return "CLAW: "+(servo2.getPosition()-servo1.getPosition())/2+". ARM: "+(servo2.getPosition()+servo1.getPosition())/2;
    }

}
