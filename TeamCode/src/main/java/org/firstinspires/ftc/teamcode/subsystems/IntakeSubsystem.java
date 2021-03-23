package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.motor.MotorSubsystem;
import com.technototes.logger.Loggable;
import com.technototes.logger.Stated;

/** Intake subsystem
 *
 */
public class IntakeSubsystem extends MotorSubsystem<Motor<?>> implements Stated<String> {
    public Motor motor;
    /** Enum for intake speed
     *
     */
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
        motor = m;
    }
    //functions for controlling intake
    public void intake(){
        motor.setSpeed(IntakeSpeed.IN.getSpeed());
    }
    public void extake() {
        motor.setSpeed(IntakeSpeed.OUT.getSpeed());
    }

    /** Returns current status of intake for logging
     *
     * @return The intake status as a String
     */
    @Override
    public String getState() {
        return motor.getSpeed() > 0 ? "INTAKING" : motor.getSpeed() < 0 ? "EXTAKING" : "IDLE";
    }

}
