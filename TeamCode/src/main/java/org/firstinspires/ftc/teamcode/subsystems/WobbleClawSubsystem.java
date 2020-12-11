package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;
import com.technototes.library.subsystem.servo.ServoSubsystem;


/** Wobble goal manipulator subsystem
 *
 */
public class WobbleClawSubsystem extends ServoSubsystem {
    public Servo clawServo;
    //claw position enum
    public enum ClawPosition {
        OPEN(0), CLOSED(1);
        double position;

        ClawPosition(double pos) {
            position = pos;
        }

        public double getPosition() {
            return position;
        }

        public WobbleClawSubsystem.ClawPosition invert() {
            return this == OPEN ? CLOSED : OPEN;
        }
    }
    public ClawPosition position;
    public WobbleClawSubsystem(Servo claw){
        super(claw);
        clawServo = claw;
        position = ClawPosition.CLOSED;
    }

    public void open(){
        clawServo.setPosition(ClawPosition.OPEN.getPosition());
        position = ClawPosition.OPEN;
    }

    public void close(){
        clawServo.setPosition(ClawPosition.CLOSED.getPosition());
        position = ClawPosition.CLOSED;

    }

    public ClawPosition getClawPosition(){
        return position;
    }
    //TODO
}
