package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
import org.firstinspires.ftc.teamcode.SummerMecRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SummerMecRobot robot = new SummerMecRobot(this);

        Pose2d startPose = new Pose2d(57, -53.5, 0);

        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(57, -29.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(9, -29.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(9, -53.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(57, -53.5))
                .build();
        while (opModeIsActive()) {
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.extendIntakeTo(540);
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
