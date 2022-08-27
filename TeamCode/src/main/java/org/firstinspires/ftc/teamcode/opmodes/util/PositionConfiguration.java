package org.firstinspires.ftc.teamcode.opmodes.util;

import static org.firstinspires.ftc.teamcode.opmodes.util.StayInPosition.stayInPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class PositionConfiguration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry);
        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoLift lift = new AutoLift(eventThread, hardwareMap);
        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
            }
        });

        waitForStart();
        eventThread.start();
        liftThread.start();

        Pose2d initial = new Pose2d(PositionConfig.x, PositionConfig.y, Math.toRadians(PositionConfig.heading));
        Pose2d lastSetPose = initial;
        drive.setPoseEstimate(initial);

        AutoLift.Positions oldSetPosition = AutoLift.Positions.INTAKING;
        AutoLift.Positions oldPosition = AutoLift.Positions.INTAKING;

        while (!isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            Pose2d pose = new Pose2d(PositionConfig.x, PositionConfig.y, Math.toRadians(PositionConfig.heading));
            if (!poseEstimate.epsilonEquals(pose) && pose != lastSetPose) {
                drive.followTrajectory(drive.trajectoryBuilder(poseEstimate)
                        .lineToLinearHeading(pose)
                        .build());
                lastSetPose = pose;
            } else if (!drive.isBusy()) {
                stayInPose(drive, pose);
            }

            if (oldSetPosition != LiftPositionConfig.position) {
                lift.setPosition(LiftPositionConfig.position);
                oldSetPosition = LiftPositionConfig.position;
            } else if (lift.getPosition() != oldPosition) {
                LiftPositionConfig.position = lift.getPosition();
            }
            oldPosition = lift.getPosition();
        }
    }

    @Config
    static class PositionConfig {
        public static double x = 0;
        public static double y = 70 - inchesToCoordinate(9);
        public static double heading = 90;
    }

    @Config
    static class LiftPositionConfig {
        public static AutoLift.Positions position = AutoLift.Positions.INTAKING;
    }
}
