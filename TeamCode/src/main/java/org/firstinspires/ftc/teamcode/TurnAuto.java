package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.math.MathContext;

@Autonomous(name = "Turn Auto", group = "Tests")
public class TurnAuto extends LinearOpMode {
    Pose2d pose;

    @Override
    public void runOpMode() {
        pose = new Pose2d(2, 4, Math.toRadians(52));

        waitForStart();

        PoseStorage.currentPose = pose;
    }
}
