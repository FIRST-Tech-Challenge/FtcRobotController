package org.firstinspires.ftc.teamcode.strafer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technototes.library.logging.Loggable;
import com.technototes.library.structure.RobotBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.strafer.subsystems.DrivebaseSubsystem;

public class Robot extends RobotBase implements Loggable {

    public Hardware hardware;

    public DrivebaseSubsystem drivebaseSubsystem;


    public Robot(HardwareMap h, Telemetry t){
        hardware = new Hardware(h);
        drivebaseSubsystem = new DrivebaseSubsystem(hardware.flMotor, hardware.frMotor, hardware.rlMotor, hardware.rrMotor);
    }
}
