package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoBotFancy extends BotFancy {
    private final MecanumDrive drivetrain;
    private Pose2d startPose;

    public AutoBotFancy(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        drivetrain = new MecanumDrive(hardwareMap, telemetry);
    }

    public void placePurplePixel() {
    }

    public MecanumDrive drivetrain() {
        return (drivetrain);
    }
}