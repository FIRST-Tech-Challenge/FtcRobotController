package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PRIMUTEST extends LinearOpMode {
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(25, 25), Math.toRadians(0))
                    .build();
            // go back
            Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end())
                    .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))
                    .build();
            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(myTrajectory);
            drive.followTrajectory(myTrajectory2);
        }
    }