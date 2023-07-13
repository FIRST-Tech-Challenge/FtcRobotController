package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "RoadRunStraightTest")
public class RoadRunStraightTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PwPRobot robot = new PwPRobot(this, false);

        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();
//7 1/8, 2
        if (isStopRequested()) return;
//        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(35.25, 57.75))
////                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
////                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
////                .lineToSplineHeading(new Pose2d(13.75, 5.75,Math.toRadians(0)))
////                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
//                .build();
//        TrajectorySequence trajSeq2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
//                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
//                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
//                .build();
        TrajectorySequence trajSeq2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(35.25, 57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25, Math.toRadians(270)), SampleMecanumDrive.getVelocityConstraint(10, 3, 51.564 / 5),
                        SampleMecanumDrive.getAccelerationConstraint(5))
                .lineToSplineHeading(new Pose2d(35.25, 57.75, Math.toRadians(270)), SampleMecanumDrive.getVelocityConstraint(10, 3, 51.564 / 5),
                        SampleMecanumDrive.getAccelerationConstraint(5))
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
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);
            robot.followTrajectorySequenceAsync(trajSeq2);

//            //func
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
