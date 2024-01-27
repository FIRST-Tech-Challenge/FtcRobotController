package org.firstinspires.ftc.teamcode.subbys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private Motor fL, fR, bL, bR;
    private RevIMU imu;

    private double mult;
    private MecanumDrive mDrive;
    public DriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, RevIMU revimu, double multiplier){
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;

        mult = multiplier;

        imu = revimu;

        mDrive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        mDrive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, true);
//        mDrive.driveFieldCentric(forwardSpeed, strafeSpeed, -turnSpeed,imu.getRotation2d().getDegrees(), false);
    }

}