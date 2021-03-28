package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.servo.ServoSubsystem;
import com.technototes.logger.Stated;

public class WobbleSubsystem extends ServoSubsystem implements Stated<String> {

    public Servo armServo;
    public Servo clawServo;

    public enum ArmPosition{
        RAISED(1), LOWERED(0.35);
        public double position;
        ArmPosition(double pos) {
            position = pos;
        }
        public double getPosition(){
            return position;
        }
    }

    public enum ClawPosition{
        OPEN(0), CLOSED(1);
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

    public WobbleSubsystem(Servo arm, Servo claw){
        super(arm, claw);
        armServo = arm;
        clawServo = claw;
        armPosition = ArmPosition.LOWERED;
        clawPosition = ClawPosition.CLOSED;
    }




    public void setClawPosition(ClawPosition pos){
        clawServo.setPosition(pos.getPosition());
        clawPosition = pos;

    }
    public void setArmPosition(ArmPosition pos){
        armServo.setPosition(pos.getPosition());
        armPosition = pos;
    }

    @Override
    public String getState() {
        return "CLAW: "+clawPosition.getPosition()+". ARM: "+armPosition.getPosition();
    }

}
