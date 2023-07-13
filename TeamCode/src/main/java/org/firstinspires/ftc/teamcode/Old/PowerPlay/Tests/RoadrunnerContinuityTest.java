package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "RoadrunnerContinuityTest")
public class RoadrunnerContinuityTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PwPRobot robot = new PwPRobot(this, false);

        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(90));
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();
//7 1/8, 2
        if (isStopRequested()) return;

        TrajectorySequenceBuilder gamer = robot.roadrun.trajectorySequenceBuilder(startPose).setReversed(true);
        Pose2d difference = new Pose2d();
        gamer.splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY()-20, startPose.getHeading()), startPose.getHeading());
        boolean[] parts = {false, false, false};
        double[] startTimes = {100, 100, 100};
        TrajectorySequenceBuilder gamer3 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(startPose.getX(), startPose.getY()-20, startPose.getHeading())).setReversed(true);
        while (opModeIsActive()) {
            if (!parts[0]) {
                parts[0] = true;
                MarkerCallback mark= new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.roadrun.breakFollowing();
                        robot.roadrun.followTrajectorySequenceAsync(gamer3.build());
                    }
                };
                TrajectorySequenceBuilder gamer2 = gamer;
                gamer2.addDisplacementMarker(0, mark);
                gamer2.splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY()-40, startPose.getHeading()), startPose.getHeading());
                robot.roadrun.followTrajectorySequenceAsync(gamer2.build());
                gamer3.splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY()-60, startPose.getHeading()), startPose.getHeading());
                startTimes[0] = getRuntime();
            }
//            if (!parts[1] && getRuntime() > startTimes[0] + 0.5) {
//                parts[1] = true;
//                robot.roadrun.breakFollowing();
//                robot.roadrun.followTrajectorySequenceAsync(gamer.build());
//                gamer.splineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY()- 60, startPose.getHeading()), startPose.getHeading());
//                startTimes[1] = getRuntime();
//            }
//            if (!parts[2] && getRuntime() > startTimes[1] + 0.5) {
//                parts[2] = true;
//                robot.roadrun.breakFollowing();
//                robot.roadrun.followTrajectorySequenceAsync(gamer.build());
//                startTimes[2] = getRuntime();
//            }
            robot.roadrun.update();
        }
    }
}
