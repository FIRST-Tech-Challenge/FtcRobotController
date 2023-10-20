package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config (value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {
    public VisionProvider visionProvider;
    private Robot robot;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        //telemetryMap.put("Current Articulation", mode );
        telemetryMap.put("Can Stage", sixCanStage);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    enum Mode {
        LINEAR, SPLINE, NO_RR, SIMPLE;
    }

    // autonomous routines
    private StateMachine left, right, blueDown, redDown,
            blueUpLinear, redUpLinear, blueDownLinear, redDownLinear,
            leftNoRR, rightNoRR, blueDownNoRR, redDownNoRR,
            blueUpSimple, redUpSimple, blueDownSimple, redDownSimple;
    // misc. routines
    public StateMachine backAndForth, square, turn, lengthTest, diagonalTest, squareNoRR, Auton;

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    public StateMachine getStateMachine(Position startingPosition, Mode mode) {
        switch(mode) {
            case LINEAR:
                switch(startingPosition) {
                    case
                            START_LEFT:
                        return left;
                    case START_RIGHT:
                        return right;
                }
                break;
            case SPLINE:
/*
                switch(startingPosition) {
                    case START_BLUE_UP:
                        return blueUpLinear;
                    case START_RED_UP:
                        return redUpLinear;
                    case START_BLUE_DOWN:
                        return blueDownLinear;
                    case START_RED_DOWN:
                        return redDownLinear;
                }
*/
                break;
            case NO_RR:
                switch(startingPosition) {
                    case START_LEFT:
                        return leftNoRR;
                    case START_RIGHT:
                        return rightNoRR;
                }
                break;
            case SIMPLE:
                /*
                switch(startingPosition) {
                    case START_BLUE_UP:
                        return blueUpSimple;
                    case START_RED_UP:
                        return redUpSimple;
                    case START_BLUE_DOWN:
                        return blueDownSimple;
                    case START_RED_DOWN:
                        return redDownSimple;
                }
                */

                break;
        }

        return null;
    }

    private StateMachine trajectorySequenceToStateMachine(TrajectorySequence trajectorySequence) {
        return Utils.getStateMachine(new Stage())
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                            trajectorySequence
                    );
                })
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    public void build(Position startingPosition) {
        //----------------------------------------------------------------------------------------------
        // Misc. Routines
        //----------------------------------------------------------------------------------------------

        TrajectorySequence backAndForthSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(24)
                        .forward(24)
                        .build();
        backAndForth = trajectorySequenceToStateMachine(backAndForthSequence);

        TrajectorySequence squareSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .build();
        square = trajectorySequenceToStateMachine(squareSequence);

        TrajectorySequence turnSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .build();
        turn = trajectorySequenceToStateMachine(turnSequence);


        //----------------------------------------------------------------------------------------------
        // Spline Routines
        //----------------------------------------------------------------------------------------------

        switch (startingPosition) {
            case START_LEFT:

                break;
            case START_RIGHT:
                break;
        }
    }


    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }
    int canScannerStage = 0;

    List<Target> uniqueTargets = new ArrayList<>();



    public int sixCanStage = 0;
    long sixCanTimer;
    public Target scoreFrom = new Target();


    int approachTargetStage = 0;

//    public boolean ApproachTarget(Target target, double stoppingDistanceInches){
//        if (target == null) return false; //this helps if ApproachTarget was called before a target was ready - could be a problem because it becomes an endless loop if a Target is never supplied
//        Pose2d robotLocation=robot.driveTrain.poseEstimate;
//        Vector2d robotVec = new Vector2d(robot.driveTrain.poseEstimate.getX(), robot.driveTrain.poseEstimate.getY());
//        double approachHeadingDegrees = Math.toDegrees( robotLocation.getHeading())+target.getCameraHeading();
//        double approachDistance = target.Distance(new Vector2d(robotLocation.getX(),robotLocation.getY()))-stoppingDistanceInches;
//        //Approach Target has two stages - first turn in place toward the target, second drive toward the target but stop short
//        switch (approachTargetStage) {
//            case 0: //turn until facing the target
//                    if (robot.driveTrain.turnUntilDegrees(approachHeadingDegrees)) {
//                        approachTargetStage++;
//                    }
//                break;
//            case 1:  //drive until we get to the right distance to deploy the gripper
//                    //this can include backing up if we are too close to the target
//                    //so it's best to approach a target from the center of the field
//                    if (robot.driveTrain.driveUntilDegrees(approachDistance, approachHeadingDegrees, 15))
//                        approachTargetStage++;
//                break;
//            case 2:
//                approachTargetStage = 0;
//                return true;
//            default:
//                approachTargetStage = 0;
//        }
//        return false;
//    }

    int approachLocationStage = 0;
//    public boolean ApproachLocation(Pose2d targetLocation, double stoppingDistanceInches){
//        if (targetLocation == null) return false; //this helps if ApproachLocation was called before a location was ready - could be a problem because it becomes an endless loop if a Target is never supplied
//        Pose2d robotLocation=robot.driveTrain.poseEstimate;
//        Vector2d robotVec = new Vector2d(robot.driveTrain.poseEstimate.getX(), robot.driveTrain.poseEstimate.getY());
//        Vector2d targetVec = new Vector2d(targetLocation.getX(), targetLocation.getY());
//        //double approachHeadingDegrees = Math.toDegrees( robotLocation.getHeading())+targetLocation.getCameraHeading();
//        double approachHeadingDegrees = robotVec.angleBetween(targetVec);
//        //double approachDistance = targetLocation.Distance(new Vector2d(robotLocation.getX(),robotLocation.getY()))-stoppingDistanceInches;
//        double approachDistance = robotVec.distTo(targetVec) - stoppingDistanceInches;
//                //ApproachLocation has two stages - first turn in place toward the location, second drive toward the location but stop short
//        switch (approachLocationStage) {
//            case 0: //turn until facing the location
//                if (robot.driveTrain.turnUntilDegrees(approachHeadingDegrees)) {
//                    approachLocationStage++;
//                }
//                break;
//            case 1:  //drive until we get to the right distance to deploy the gripper
//                //this can include backing up if we are too close to the location
//                //so it's best to approach a location from the center of the field
//                if (robot.driveTrain.driveUntilDegrees(approachDistance, approachHeadingDegrees, 15))
//                    approachLocationStage++;
//                break;
//            case 2:
//                approachLocationStage = 0;
//                return true;
//            default:
//                approachLocationStage = 0;
//        }
//        return false;
//    }
}
