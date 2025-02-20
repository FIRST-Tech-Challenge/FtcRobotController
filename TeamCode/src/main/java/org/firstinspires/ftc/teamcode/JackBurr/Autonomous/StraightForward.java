package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.nio.PathFileManager;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;

@Autonomous
@Config
public class StraightForward extends LinearOpMode {
    public Follower follower;
    public boolean traj1followed = false;
    public PathChain scorePreloadChain;
    public static Pose startPose = new Pose(38, 62, Math.toRadians(270));
    public static Pose bucket = new Pose(38, 32, Math.toRadians(270));

    public Path scorePreload;


    public int step = 1;


    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        slides.init(hardwareMap);
        follower = new Follower(hardwareMap);
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(bucket)));
        scorePreloadChain = follower.pathBuilder()
                .addPath(scorePreload)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addTemporalCallback(0, ()->{
                    slides.runLeftSlideToPositionPID(constants.LEFT_SLIDE_HIGH_BASKET);
                    slides.runRightSlideToPositionPID(constants.RIGHT_SLIDE_HIGH_BASKET);
                })
                .build();
        waitForStart();
        while(opModeIsActive()) {
            follower.update();
            if (step == 1) {
                if(!traj1followed && !follower.isBusy()) {
                    follower.followPath(scorePreloadChain, true);
                    follower.update();
                    traj1followed = true;
                }
                else if(traj1followed && !follower.isBusy()) {
                    step = 2;
                    traj1followed = false;
                }
            }
            if (step == 2) {
                if(!follower.isBusy() && !traj1followed) {
                    follower.turnTo(bucket.getHeading());
                    traj1followed = true;
                }
            }
            if (isStopRequested()) {
                return;
            }
        }

    }

}
