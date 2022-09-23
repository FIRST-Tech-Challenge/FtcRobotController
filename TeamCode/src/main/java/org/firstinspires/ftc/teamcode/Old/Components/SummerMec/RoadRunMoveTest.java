package org.firstinspires.ftc.teamcode.Old.Components.SummerMec;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SummerMecRobot robot = new SummerMecRobot(this);

        Pose2d startPose = new Pose2d(-35, -68, Math.toRadians(90));
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, -34.5-7,Math.toRadians(90)))
                .lineTo(new Vector2d(-35,-11-7))
                .lineToSplineHeading(new Pose2d(-33, 12.5-7,Math.toRadians(0)))
                .lineTo(new Vector2d(-10.5, 12.5-7))
                .lineToSplineHeading(new Pose2d(13, 11-7,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(13, -13.5-7,Math.toRadians(0)))
                .lineTo(new Vector2d(36.5, -14-7))
                .lineToSplineHeading(new Pose2d(57, -14.5-7,Math.toRadians(-90)))
                .lineTo(new Vector2d(57, -34.5-7))
                .lineToSplineHeading(new Pose2d(57, -58-7,Math.toRadians(-180)))
                .lineTo(new Vector2d(-10.5, -58-7))
                .lineToSplineHeading(new Pose2d(-36, -58-7,Math.toRadians(90)))
                .build();

        while (opModeIsActive()) {
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
