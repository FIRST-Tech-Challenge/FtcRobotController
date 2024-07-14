package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.DrivetrainMecanumRR;

public class AutoBotFancy extends BotFancy {
    private final DrivetrainMecanumRR drivetrain;
    private Pose2d startPose;

    public AutoBotFancy(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        drivetrain = new DrivetrainMecanumRR(hardwareMap, telemetry);
    }

    public void placePurplePixel() {
    }

    public DrivetrainMecanumRR drivetrain() {
        return (drivetrain);
    }
}