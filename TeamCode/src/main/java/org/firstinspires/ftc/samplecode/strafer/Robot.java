package org.firstinspires.ftc.samplecode.strafer;

import com.technototes.logger.Loggable;

import org.firstinspires.ftc.samplecode.strafer.subsystems.DrivebaseSubsystem;

public class Robot implements Loggable {

    public Hardware hardware;

    public DrivebaseSubsystem drivebaseSubsystem;



    public Robot(){
        hardware = new Hardware();
        drivebaseSubsystem = new DrivebaseSubsystem(hardware.flMotor, hardware.frMotor, hardware.rlMotor, hardware.rrMotor, hardware.rangeSensor);
    }
}
