package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;

/** Drivebase subsystem
 *
 */
public class DrivebaseSubsystem extends MecanumDrivebaseSubsystem<Motor<?>> {
    public IMU imu;
    public DrivebaseSubsystem(Motor flMotor, Motor frMotor, Motor rlMotor, Motor rrMotor, IMU i) {
        super(i::gyroHeading, flMotor, frMotor, rlMotor, rrMotor);
        imu = i;
    }

    //TODO
}
