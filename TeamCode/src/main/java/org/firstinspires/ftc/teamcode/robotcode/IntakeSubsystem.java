package org.firstinspires.ftc.teamcode.robotcode;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.motor.MotorSubsystem;

import org.firstinspires.ftc.teamcode.examples.clawbot.subsystems.ClawSubsystem;

public class IntakeSubsystem extends MotorSubsystem<Motor<?>> {
    public enum IntakeSpeed{
        IN(1), OUT(-1);
        double speed;
        IntakeSpeed(double s){
            speed = s;
        }
        public double getSpeed(){
            return speed;
        }

    }
    public IntakeSubsystem(Motor m) {
        super(m);
    }
    public void intake(){
        setSpeed(IntakeSpeed.IN.getSpeed());
    }
    public void extake() {
        setSpeed(IntakeSpeed.OUT.getSpeed());
    }

}
