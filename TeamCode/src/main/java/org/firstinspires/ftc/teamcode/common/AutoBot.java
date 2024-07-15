package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

public class AutoBot extends Bot {
    private final SampleMecanumDrive drivetrain;
    private Pose2d startPose;
    public AutoBot(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn)
    {
        super(hardwareMap, telemetry);
        drivetrain = new SampleMecanumDrive(hardwareMap,telemetry);
    }

    public void placePurplePixel()
    {
    }

    public SampleMecanumDrive drivetrain()
    {
        return(drivetrain);
    }
}