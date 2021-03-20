package org.firstinspires.ftc.samplecode.strafer.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.samplecode.strafer.odometry.DeadWheelOdometry;

public class DrivebaseSubsystem extends MecanumDrivebaseSubsystem<EncodedMotor<DcMotor>> implements Loggable {
////    public enum
    public DeadWheelOdometry odometry;
    public DrivebaseSubsystem(EncodedMotor<DcMotor> flMotor, EncodedMotor<DcMotor> frMotor, EncodedMotor<DcMotor> rlMotor, EncodedMotor<DcMotor> rrMotor) {
        super(flMotor, frMotor, rlMotor, rrMotor);
        //rrMotor.getEncoder().getPosition() not null
        odometry = new DeadWheelOdometry(()->rrMotor.getEncoder().getAsDouble()/-1880, ()->frMotor.getEncoder().getAsDouble()/1880, ()->rlMotor.getEncoder().getAsDouble()/-1880);
    }


}
