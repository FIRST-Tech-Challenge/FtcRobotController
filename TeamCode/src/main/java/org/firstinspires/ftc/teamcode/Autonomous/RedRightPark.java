package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous
public class RedRightPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(15.5, -56, Math.toRadians(-90)));
        int pos = 0;
        TrajectorySequence park = robot.roadrun.trajectorySequenceBuilder(new Pose2d(15.5, -56, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(25.5, -50, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(55.5, -60, Math.toRadians(180)))
                .build();
        while(!isStarted()){
//            pos=0;
        } //willy phone password 272714
        while(!isStopRequested()&&opModeIsActive()&&!robot.queuer.isFullfilled()){
            robot.followTrajSeq(park);
            robot.queuer.setFirstLoop(false);
            robot.update();
        }
    }
}
