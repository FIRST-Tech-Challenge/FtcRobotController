package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.vision.TurretCamera;

import java.util.Arrays;
import java.util.List;

public class Robot {

    public final Pinpoint pinpoint;
    public final MecanumDrive drive;
    public final TurretCamera turretcam;
    private final List<Subsystem> subsystems;

    public Robot(HardwareMap hardwareMap){
        pinpoint = new Pinpoint(hardwareMap);
        drive =  new MecanumDrive(hardwareMap, pinpoint);
        turretcam = new TurretCamera(hardwareMap);
        //REMINDER: add subsystems to this list when they're created.
        subsystems = Arrays.asList(drive, pinpoint, turretcam);
    }

    public void periodic(){
        pinpoint.periodic();
        drive.periodic();
        turretcam.periodic();
    }

    public void writeTelemetry(Telemetry telemetry){
        for (Subsystem a : subsystems){
            a.writeTelemetry(telemetry);
        }
    }
    public void stop(){
        turretcam.shutdown();
    }
}
