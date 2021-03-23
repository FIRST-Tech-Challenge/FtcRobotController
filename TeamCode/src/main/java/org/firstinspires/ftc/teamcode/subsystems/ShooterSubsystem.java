package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;
import com.technototes.logger.Stated;

/** Shooter subsystem
 *
 */
public class ShooterSubsystem extends EncodedMotorSubsystem implements Stated<Double> {
    public EncodedMotor motor;
    public ShooterSubsystem(EncodedMotor m){
        super(m);
        motor = m;

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

    @Override
    public Double getState() {
        return getVelocity();
    }
}
