package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

//@Disabled

/**
 * Warren Zhou
 * 9/6/23
 * Odom localizer 8 24x24 squares
 */
@Autonomous(name = "AprilTagRRTest")
public class AprilTagRRTest extends LinearOpMode {
    Queuer queuer;
    SampleMecanumDrive roadrun;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        roadrun = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
        Pose2d startPose = new Pose2d(20.5, -36, Math.toRadians(180));
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CVMaster cv = new CVMaster();
        cv.switchToApril();
        roadrun.setPoseEstimate(startPose);
        queuer = new Queuer();
        int loops = 0;

        waitForStart();
        if (isStopRequested()) return;
        TrajectorySequence trajSeq2 = roadrun.trajectorySequenceBuilder(new Pose2d(20.5, -36, Math.toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, -36, toRadians(-180)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(20.5, -36, Math.toRadians(180)))
                        .build();

        //        while (opModeIsActive()) {
        resetRuntime();
        BasicRobot.time = 0;
        while(!isStopRequested()&&opModeIsActive()) {
            for(int i=0;i<20;i++){
                followTrajAsync(trajSeq2);
            }
            loops++;
            packet.put("loopTime", loops/BasicRobot.time);
            queuer.setFirstLoop(false);
            robot.update();
            roadrun.update();
           cv.update();
        }
    }
    public void followTrajAsync(TrajectorySequence traj){
        if(queuer.queue(false, queuer.isStarted()&&!roadrun.isBusy())){
            if(!roadrun.isBusy()) {
                roadrun.followTrajectorySequenceAsync(traj);
            }
        }
    }

}
