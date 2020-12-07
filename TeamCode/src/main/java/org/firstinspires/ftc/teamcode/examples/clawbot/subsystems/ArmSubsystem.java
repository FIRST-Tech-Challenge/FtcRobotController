package org.firstinspires.ftc.teamcode.examples.clawbot.subsystems;

import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;

public class ArmSubsystem extends EncodedMotorSubsystem {

    public ArmSubsystem(EncodedMotor motor){
        super(motor);
    }
    //custom function
    public boolean setArmPosition(double pos){
        return setPosition(pos);
    }
}
    