package org.firstinspires.ftc.teamcode.strafer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.strafer.subsystems.DrivebaseSubsystem;

public class Robot implements Loggable {

    public Hardware hardware;

    public DrivebaseSubsystem drivebaseSubsystem;



    public Robot(){
        hardware = new Hardware();
        drivebaseSubsystem = new DrivebaseSubsystem(hardware.flMotor, hardware.frMotor, hardware.rlMotor, hardware.rrMotor, hardware.rangeSensor);
    }
}
