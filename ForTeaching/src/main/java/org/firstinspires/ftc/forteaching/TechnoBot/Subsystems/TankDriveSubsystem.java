package org.firstinspires.ftc.forteaching.TechnoBot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.drivebase.TankDrivebaseSubsystem;

public class TankDriveSubsystem extends TankDrivebaseSubsystem<DcMotorEx> {

    /**
     * Create tank drivebase
     *
     * @param leftMotor  The motor/motorgroup for the left side of the drivebase
     * @param rightMotor The motor/motorgroup for the right side of the drivebase
     */
    public TankDriveSubsystem(Motor<DcMotorEx> leftMotor, Motor<DcMotorEx> rightMotor) {
        super(leftMotor, rightMotor);
    }
}
