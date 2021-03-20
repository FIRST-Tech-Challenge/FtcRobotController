package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;

/** Shooter subsystem
 *
 */
public class ShooterSubsystem extends EncodedMotorSubsystem {
    //TODO fix encoded motor to properly use velocity control
    public EncodedMotor<DcMotor> motor1, motor2;
    private DcMotor internal1, internal2;
    public ShooterSubsystem(EncodedMotor<DcMotor> m1, EncodedMotor<DcMotor> m2){
        super(m1, m2);
        motor1 = m1;
        motor2 = m2;
        internal1 = motor1.getDevice();
        internal1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        internal2 = motor2.getDevice();
        internal2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setVelocity(double ticksPerSecond){
        internal1.setPower(ticksPerSecond);
        internal2.setPower(ticksPerSecond);
    }
    public double getVelocity(){
        return internal1.getPower();
    }
    public double getIdleVelocity(){
        return 100; //idle speed here
    }
    public boolean isAtIdleVelocity(){
        return getIdleVelocity() <= getVelocity();
    }
}
