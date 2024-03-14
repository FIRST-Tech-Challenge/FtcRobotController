package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

//@Disabled
@Config
@Autonomous
public class IntakeAutoTest extends LinearOpMode {
    public static int height=5;
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
    robot.roadrun.setPoseEstimate(new Pose2d(-55, 0, toRadians(-180)));
        TrajectorySequence run = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-55,0, toRadians(-180))).
                back(10).build();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            robot.resetAuto();
            robot.intakeAuto(height);
            robot.queuer.waitForFinish();
            robot.followTrajSeq(run);
            robot.update();
            robot.queuer.setFirstLoop(false);
        }
    }
}
