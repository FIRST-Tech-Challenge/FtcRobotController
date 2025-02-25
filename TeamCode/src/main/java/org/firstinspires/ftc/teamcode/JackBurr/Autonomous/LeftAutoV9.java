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
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.nio.PathFileManager;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;

@Autonomous
@Config
public class LeftAutoV9 extends LinearOpMode {
    public Follower follower;
    public boolean traj1followed = false;
    public PathChain scorePreloadChain;
    public static Pose startPose = new Pose(38, 62, Math.toRadians(270));
    public static Pose bucket = new Pose(50, 52, Math.toRadians(-135));

    public Path scorePreload;


    public int step = 1;

    public ElapsedTime stepTimer = new ElapsedTime();

    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        slides.init(hardwareMap);
        deliveryAxon.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        scorePreload = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), new Point(bucket.getX(), bucket.getY(), Point.CARTESIAN)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading());
        scorePreloadChain = follower.pathBuilder()
                .addPath(scorePreload)
                .addTemporalCallback(0, ()->{
                    slides.runBothSlidesToNegatedPositions(constants.RIGHT_SLIDE_HIGH_BASKET);
                    telemetry.addData("Follower Busy: ", follower.isBusy());
                    telemetry.addData("At End: ", follower.atParametricEnd());
                })
                .addTemporalCallback(200, ()->{
                    deliveryAxon.setPosition(constants.DELIVERY_UP);
                })
                .addTemporalCallback(5000, ()->{
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    follower.breakFollowing();
                })

                .build();
        waitForStart();
        stepTimer.reset();
        //follower.followPath(scorePreloadChain);
        while(opModeIsActive()) {
            if (isStopRequested()) {
                return;
            }
            follower.update();
            telemetry.update();
            follower.drawOnDashBoard();
            telemetry.addLine("Step: " + step);
            if (step == 1) {
                if(!traj1followed) {
                    follower.followPath(scorePreloadChain);
                    traj1followed = true;
                }
                telemetry.addData("Pose: ", follower.getPose());
                telemetry.addData("Is done: ", follower.atParametricEnd());
                if(stepTimer.seconds() > 5){
                    follower.breakFollowing();
                    stepTimer.reset();
                    step = 2;
                }
            }
            if (step == 2) {
                while (stepTimer.seconds() < 5){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    if (isStopRequested()) {
                        return;
                    }
                }
                if(stepTimer.seconds() > 5){
                    stepTimer.reset();
                    step = 3;
                }
            }
            if(step == 3){
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                if(stepTimer.seconds() > 3){
                    stepTimer.reset();
                    step = 4;
                }
            }
            if(step == 4){
                if(stepTimer.seconds() < 2) {
                    slides.runBothSlidesToNegatedPositions(0);
                }
                else {
                    step = 5;
                }

            }
        }

    }

}
