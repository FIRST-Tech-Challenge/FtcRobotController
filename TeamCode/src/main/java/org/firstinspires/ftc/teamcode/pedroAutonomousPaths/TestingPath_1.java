package org.firstinspires.ftc.teamcode.pedroAutonomousPaths;

//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.util.Constants;


import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;

@Autonomous(name = "Testing Path Around The Field")
public class TestingPath_1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(10, 33, Math.toRadians(0));

    private final Pose farRightPose = new Pose(118, 33, Math.toRadians(90));

    private final Pose farLeftPose = new Pose(118, 118, Math.toRadians(180));

    private final Pose nearLeftPose = new Pose(27,118, Math.toRadians(270));

    private final Pose nearRightPose = new Pose(27, 33, Math.toRadians(0));

    private Path driveFarRight, driveFarLeft, driveNearLeft, driveNearRight;

    public void buildPaths() {
        driveFarRight = new Path(new BezierLine(new Point(startPose), new Point(farRightPose)));
        driveFarRight.setLinearHeadingInterpolation(startPose.getHeading(), farRightPose.getHeading());

        driveFarLeft = new Path(new BezierLine(new Point(farRightPose), new Point(farLeftPose)));
        driveFarLeft.setLinearHeadingInterpolation(farRightPose.getHeading(), farLeftPose.getHeading());

        driveNearLeft = new Path(new BezierLine(new Point(farLeftPose), new Point(nearLeftPose)));
        driveNearLeft.setLinearHeadingInterpolation(farLeftPose.getHeading(), nearLeftPose.getHeading());

        driveNearRight = new Path(new BezierLine(new Point(nearLeftPose), new Point(nearRightPose)));
        driveNearRight.setLinearHeadingInterpolation(nearLeftPose.getHeading(), nearRightPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(driveFarRight);
                setPathValue(1);
                break;

            case 1:
                if(follower.getPose().getX() > (farRightPose.getX() - 1) && follower.getPose().getY() > (farRightPose.getY() - 1)) {
                    follower.followPath(driveFarLeft);
                    setPathValue(2);
                }
                break;

            case 2:
                if(follower.getPose().getX() > (farLeftPose.getX() - 1) && follower.getPose().getY() > (farLeftPose.getY() - 1)) {
                    follower.followPath(driveNearLeft);
                    setPathValue(3);
                }
                break;

            case 3:
                if(follower.getPose().getX() > (nearLeftPose.getX() - 1) && follower.getPose().getY() > (nearLeftPose.getY() - 1)) {
                    follower.followPath(driveNearRight);
                    setPathValue(-1);
                }
                break;
        }
    }

    public void setPathValue(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();


    }
}
