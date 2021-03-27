package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.subsystem.drivebase.EncodedMecanumDrivebaseSubsystem;
import com.technototes.logger.Stated;

/** Drivebase subsystem
 *
 */
public class OldDrivebaseSubsystem extends EncodedMecanumDrivebaseSubsystem implements Stated<OldDrivebaseSubsystem.DriveSpeed> {
    public IMU imu;


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
            return speed;
        }
    }

    public DriveSpeed driveSpeed;

    public OldDrivebaseSubsystem(EncodedMotor flMotor, EncodedMotor frMotor, EncodedMotor rlMotor, EncodedMotor rrMotor, IMU i) {
        //note order
        super(i::gyroHeading, flMotor, frMotor, rlMotor, rrMotor);
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
        return driveSpeed.getSpeedAsVelocity();
    }


    @Override
    public DriveSpeed getState() {
        return getDriveSpeed();
    }
    //TODO rest of subsystem
    //TODO make turn value custom

    //test
    @Override
    public void drive(double speed, double angle, double rotation) {
        double x = Math.cos(angle) * speed;
        double y = Math.sin(angle) * speed;

        double powerCompY = -(x + y);
        double powerCompX = x - y;

        speed = Range.clip(speed + Math.abs(rotation), 0, 1);

        double flPower = powerCompY - powerCompX - 2*rotation;
        double frPower = -powerCompY - powerCompX - 2*rotation;
        double rlPower = powerCompY + powerCompX - 2*rotation;
        double rrPower = -powerCompY + powerCompX - 2*rotation;

        double scale = getScale(flPower, frPower, rlPower, rrPower);
        scale = scale == 0 ? 0 : speed/scale;
        scale = Math.cbrt(scale);
        drive(flPower*scale, frPower*scale,rlPower*scale, rrPower*scale);
    }

}
