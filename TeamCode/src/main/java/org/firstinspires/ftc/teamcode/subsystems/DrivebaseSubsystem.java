package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import org.firstinspires.ftc.teamcode.subsystems.GyroSensor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.subsystem.drivebase.EncodedMecanumDrivebaseSubsystem;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;
import com.technototes.logger.Stated;

/** Drivebase subsystem
 *
 */
public class DrivebaseSubsystem extends MecanumDrivebaseSubsystem<Motor<?>> implements Stated<DrivebaseSubsystem.DriveSpeed> {
    public GyroSensor imu;


    //drivespeed stuff
    public enum DriveSpeed{
        NORMAL(0.7), SNAIL(0.2), TURBO(1);
        double speed;
        DriveSpeed(double s) {
            speed = s;
        }
        double getSpeed(){
            return speed;
        }
        //for velocity control
        double getSpeedAsVelocity(){
            return 2655*speed;
        }
    }

    public DriveSpeed driveSpeed;

    public DrivebaseSubsystem(Motor flMotor, Motor frMotor, Motor rlMotor, Motor rrMotor, GyroSensor i) {
        //note order
        super(()->0, frMotor, flMotor, rrMotor, rlMotor);
        driveSpeed = DriveSpeed.NORMAL;
        imu = i;
    }
    public DriveSpeed getDriveSpeed() {
        return driveSpeed;
    }

    public void setDriveSpeed(DriveSpeed driveSpeed) {
        this.driveSpeed = driveSpeed;
    }

    @Override
    public double getSpeed() {
        return driveSpeed.getSpeed();
    }


    @Override
    public DriveSpeed getState() {
        return getDriveSpeed();
    }
    //TODO rest of subsystem
}
