package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            Trajectory forward = drive.trajectoryBuilder(drive.pose)
                    .forward(DISTANCE)
                    .build();

            Trajectory backward = drive.trajectoryBuilder(forward, true)
                    .forward(-DISTANCE)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                drive.followTrajectory(forward).runBlocking();
                drive.followTrajectory(backward).runBlocking();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            Trajectory forward = drive.trajectoryBuilder(drive.pose)
                    .forward(DISTANCE)
                    .build();

            Trajectory backward = drive.trajectoryBuilder(forward, true)
                    .forward(-DISTANCE)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                drive.followTrajectory(forward).runBlocking();
                drive.followTrajectory(backward).runBlocking();
            }
        } else {
            throw new AssertionError();
        }
    }
}
