package org.firstinspires.ftc.teamcode.pedroPathing;

import com.google.ar.core.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
@Autonomous
public class CodyPath  extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        SHOOT_PRELOAD, DRIVE_STARTPOS_SHOOT_POS


    }

    PathState pathState;

    private final Pose startPose = new Pose(57.33490566037736, 8.133490566037743, Math.toRadians());
    private final Pose shootPose = new Pose(57.60188679245283,32.79622641509434, Math.toRadians());

    private final Pose endPose = new Pose();

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getZAxis(), shootPose.getZAxis())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    telemetry.addLine();
                }
                break;
            default:
                telemetry.addLine();
                break;

        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer() ;
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }
    @Override
    public void start() {
      opModeTimer.resetTimer();
      setPathState(pathState);

    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());



    }
        }


