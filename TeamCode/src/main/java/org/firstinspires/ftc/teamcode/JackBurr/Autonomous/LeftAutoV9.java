package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.nio.PathFileManager;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;

@Autonomous
@Config
public class LeftAutoV9 extends LinearOpMode {
    public Follower follower;
    public PathChain scorePreloadChain;
    public static Pose startPose = new Pose(38, 62, Math.toRadians(270));
    public static Pose bucket = new Pose(50, 52, Math.toRadians(45));

    public Path scorePreload;


    public int step = 1;


    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();

    @Override
    public void runOpMode() throws InterruptedException {
        slides.init(hardwareMap);
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(bucket)));
        scorePreloadChain = follower.pathBuilder()
                .addPath(scorePreload)
                .setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading())
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPositionPID(constants.LEFT_SLIDE_HIGH_BASKET);
                    slides.runRightSlideToPositionPID(constants.RIGHT_SLIDE_HIGH_BASKET);
                })
                .build();
        waitForStart();
        if(step == 1) {
            follower.followPath(scorePreloadChain, true);
            step = 2;
        }
        if(step == 2){

        }
        if(isStopRequested()){
            return;
        }

    }

}
