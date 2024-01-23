package org.firstinspires.ftc.teamcode.Autonomus.Road;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomus.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomus.methods.ArmControl;

@Autonomous(name = "TestAuto", group = "Red")
public class TestAuto extends LinearOpMode {

    public ArmControl ArmControl = new ArmControl();

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        ArmControl.initHW(this);

        Trajectory trajSt = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    ArmControl.setArm(this, 500);
                })
                .build();

        //1
        Trajectory traj0 = drive.trajectoryBuilder(trajSt.end())
                .lineToConstantHeading(new Vector2d(-32, 0))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToConstantHeading(new Vector2d(-36, 0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                .addTemporalMarker(2, () -> {
                    ArmControl.setArm(this, 300);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())     //come to stack
                .lineToConstantHeading(new Vector2d(-60, -12))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(-48, -12))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(0)))
                .build();

        //2
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToConstantHeading(new Vector2d(-32, 0))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToConstantHeading(new Vector2d(-36, 0))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToConstantHeading(new Vector2d(-60, -12))
                .build();

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToConstantHeading(new Vector2d(-48, -12))
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(0)))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        //start auto


        ArmControl.setCatch(0.75);     //catch pre-loaded cone

        drive.followTrajectory(trajSt);     //move to high junction

        drive.followTrajectory(traj1);      //come closer

        ArmControl.setCatch(0.66);     //drop pre-loaded cone

        drive.followTrajectory(traj2);      //move away

        ArmControl.setArm(this, 300);

        drive.followTrajectory(traj3);      //come closer to stack

        drive.followTrajectory(traj4);      //move away from the stack

        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);

    }
}