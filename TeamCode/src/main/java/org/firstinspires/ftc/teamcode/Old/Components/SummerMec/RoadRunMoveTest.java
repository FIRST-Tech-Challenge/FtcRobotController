package org.firstinspires.ftc.teamcode.Old.Components.SummerMec;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SummerMecRobot robot = new SummerMecRobot(this);

        Pose2d startPose = new Pose2d(41, 61, Math.toRadians(270));
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35.25, 57.75))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();
        TrajectorySequence trajSeq2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();

        while (opModeIsActive()) {
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
