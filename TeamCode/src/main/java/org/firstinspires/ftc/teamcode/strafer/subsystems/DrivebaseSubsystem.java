package org.firstinspires.ftc.teamcode.strafer.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DrivebaseSubsystem extends MecanumDrivebaseSubsystem {
//    public enum
    public RangeSensor sensor;
    public DrivebaseSubsystem(Motor flMotor, Motor frMotor, Motor rlMotor, Motor rrMotor, RangeSensor s) {
        super(flMotor, frMotor, rlMotor, rrMotor);
        sensor = s;
    }

    public double getDistance(){
        return sensor.getSensorValue(DistanceUnit.INCH);
    }

    @Override
    public double getSpeed() {
        return 0;
    }

}
