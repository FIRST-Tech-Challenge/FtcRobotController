package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team5898.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.team5898.pedroPathing.constants.LConstants;

@Autonomous(name = "Square Around The Field", group = "Examples")
public class SquareAroundField extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 80, Math.toRadians(0));

    /** First point - moving to x: 120, y: 24 */
    private final Pose point1 = new Pose(31, 72, Math.toRadians(90));

    /** Second point - moving to x: 120, y: 120 */
    private final Pose point2 = new Pose(30.5, 34, Math.toRadians(180));

    /** Third point - moving to x: 24, y: 120 */
    private final Pose point3 = new Pose(73, 34, Math.toRadians(270));

    /** Final parking pose - moving to x: 73, y: 26 */
    private final Pose point4 = new Pose(109, 117.5 , Math.toRadians(90));

    /** Park Control Pose for curve manipulation */
    private final Pose point4control = new Pose(142, 25, Math.toRadians(90));

    /* PathChains */
    private PathChain line1, line2, line3, line4;

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(point1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), point1.getHeading())
                .build();

       line2 = follower.pathBuilder()
               .addPath(new BezierLine(new Point(point1), new Point(point2)))
               .setLinearHeadingInterpolation(point1.getHeading(), point2.getHeading())
               .build();
       line3 = follower.pathBuilder()
               .addPath(new BezierLine(new Point(point2), new Point(point3)))
               .setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading())
               .build();
       line4 = follower.pathBuilder()
               .addPath(new BezierCurve(new Point(point3),new Point(point4control),new Point(point4)))
               .setLinearHeadingInterpolation(point3.getHeading(), point4.getHeading())
               .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}