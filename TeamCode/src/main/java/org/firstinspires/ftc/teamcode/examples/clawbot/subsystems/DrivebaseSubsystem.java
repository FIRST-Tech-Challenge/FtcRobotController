package org.firstinspires.ftc.teamcode.examples.clawbot.subsystems;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.drivebase.TankDrivebaseSubsystem;

//drivebase subsystem
public class DrivebaseSubsystem extends TankDrivebaseSubsystem<Motor<?>> {
    //subsystem enums
    public enum DriveSpeed{
        SNAIL(0.3), NORMAL(0.5), TURBO(1);
        double speed;
        DriveSpeed(double s){
            speed = s;
        }
        double getSpeed(){
            return speed;
        }
    }
    public DriveSpeed speed;
    public DrivebaseSubsystem(Motor l, Motor r) {
            super(l, r);
        speed = DriveSpeed.NORMAL;
    }
    //custom function to spin the robot
    public void spin(){
        drive(0.5, -0.5);
    }

    //use the new driveSpeed enum
    @Override
    public double getSpeed() {
        return speed.getSpeed();
    }
}
