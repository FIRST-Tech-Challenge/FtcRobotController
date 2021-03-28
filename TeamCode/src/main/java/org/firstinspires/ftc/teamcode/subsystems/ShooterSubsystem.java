package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;
import com.technototes.logger.Stated;

/** Shooter subsystem
 *
 */
public class ShooterSubsystem extends EncodedMotorSubsystem implements Stated<Double> {
    public EncodedMotor motor;
    public Servo servo;
    public ShooterSubsystem(EncodedMotor m, Servo s){
        super(m);
        motor = m;
        servo = s;
        s.setRange(0.5, 1);

    }
    public void setVelocity(double ticksPerSecond){
        motor.setVelocity(ticksPerSecond);
    }
    public double getVelocity(){
        return motor.getVelocity();
    }
    public double getIdleVelocity(){
        return 500; //idle speed here
    }
    public boolean isAtIdleVelocity(){
        return getIdleVelocity() <= getVelocity();
    }

    public void setFlapPosition(double pos){
        servo.setPosition(pos);
    }

    public double getFlapPosition(){
        return servo.getPosition();
    }

    @Override
    public Double getState() {
        return getVelocity();
    }
}
