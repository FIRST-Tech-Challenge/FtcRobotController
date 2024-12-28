package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "RoadRunner Test")
public class RoadRunnerTestAuto extends LinearOpMode {
    private MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Path path = new PathBuilder(new Pose2d(0, 0, 0))
                .strafeTo()
                .lineTo(new Vector2d(30, 15))
                .build();


    }

}