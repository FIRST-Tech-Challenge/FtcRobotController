package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PwPRobot robot = new PwPRobot(this, false);

        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();
//1.5,0,0.25,0.7,0.4,0.8,0.9,1.3,0.5,0,0.25
        if (isStopRequested()) return;
//        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(35.25, 57.75))
////                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
////                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
////                .lineToSplineHeading(new Pose2d(13.75, 5.75,Math.toRadians(0)))
////                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
//                .build();
//        TrajectorySequence trajSeq2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
////                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
////                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
//                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(1.75, 57.75))
//                .lineTo(new Vector2d(35.25, 57.75))
////                .build();
        TrajectorySequence trajSeq2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();

        while (opModeIsActive()) {
//            robot.followTrajectorySequenceAsync(trajSeq);
            //extendFunction
            //.2,.4
            //1,5/8,2.5,-1.125,2.75,2.5
            //1,.5,1,0,1,0
            //0,5/8,1/4,-1,1.25
            //0,0,1/2,1/4,1.25
            //2.8,0,2.5
            //1.5,0,0.8
            //2
            //0.5,0.5,0.25,0.5
            //2
            //1
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);

//            //func
            robot.setFirstLoop(false);
            robot.roadrun.update();
            telemetry.update();
        }
    }
}
