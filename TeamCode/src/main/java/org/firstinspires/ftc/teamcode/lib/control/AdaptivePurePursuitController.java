package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AdaptivePurePursuitController {
    private static final double DISTANCE_TO_FINAL_POINT_THRESHOLD = 2d; //in
    private static final double kP_TURN = 1 / Math.toRadians(180d);
    private LinkedList<CurvePoint> path;
    private boolean reachedGoal;

    public AdaptivePurePursuitController(CurvePoint... pathSegments) {
        setPath(new LinkedList<>());
        getPath().addAll(Arrays.asList(pathSegments));
        setReachedGoal(false);
    }

    public void resetPath(CurvePoint... pathSegments) {
        getPath().clear();
        getPath().addAll(Arrays.asList(pathSegments));
        setReachedGoal(false);
    }

    public Pose2d follow(Supplier<Pose2d> robotPose, DoubleSupplier robotSpeed) {
        return follow(robotPose.get(), robotSpeed.getAsDouble());
    }

    public Pose2d follow(Pose2d robotPose, double robotSpeed) {
        if(getPath().size() == 0) {
            setReachedGoal(true);
            return null;
        } else if(getPath().size() == 1) {
            CurvePoint finalCurvePoint = getPath().get(0);
            double distanceToDestination = finalCurvePoint.getPose().getTranslation().distance(robotPose.getTranslation());
            if(distanceToDestination <= getDistanceToFinalPointThreshold()) {
                setReachedGoal(true);
                //getPath().remove();
                //follow(robotPose, robotSpeed);
            } else {
                setReachedGoal(false);
            }

            double lookaheadDistance = finalCurvePoint.getLookaheadDistance(robotSpeed);
            if(distanceToDestination < lookaheadDistance) {
                lookaheadDistance = distanceToDestination;
            }

            Translation2d robotToDestination = finalCurvePoint.getPose().getTranslation().translateBy(robotPose.getTranslation().inverse());
            double fieldRelativeAngleToDestination = Math.atan2(robotToDestination.y(), robotToDestination.x());
            double normalization = lookaheadDistance / finalCurvePoint.getLookaheadDistance(robotSpeed);
            Translation2d translationPower = new Translation2d(normalization * Math.sin(fieldRelativeAngleToDestination),
                    -normalization * Math.cos(fieldRelativeAngleToDestination));
            Rotation2d headingError = finalCurvePoint.getPose().getRotation().rotateBy(robotPose.getRotation().inverse());
            if(Math.abs(headingError.getRadians() + 2d * Math.PI) < Math.abs(headingError.getRadians())) {
                headingError = headingError.rotateBy(new Rotation2d(2d * Math.PI, false));
            } else if(Math.abs(headingError.getRadians() - 2d * Math.PI) < Math.abs(headingError.getRadians())) {
                headingError = headingError.rotateBy(new Rotation2d(-2d * Math.PI, false));
            }

            Rotation2d rotationPower = headingError.multiply(getkP_TURN(), false);
            return new Pose2d(translationPower, rotationPower);
        } else {
            CurvePoint lastCurvePoint = getPath().get(0);
            CurvePoint nextCurvePoint = getPath().get(1);
            Translation2d lastPointToRobot = robotPose.getTranslation().translateBy(lastCurvePoint.getPose().getTranslation().inverse());
            Translation2d pathDirection = nextCurvePoint.getPose().getTranslation().translateBy(lastCurvePoint.getPose().getTranslation().inverse());
            Translation2d pathUnitVector = pathDirection.scale(1 / pathDirection.norm());
            if(pathDirection.norm() - Translation2d.dot(pathUnitVector, lastPointToRobot) <= nextCurvePoint.getLookaheadDistance(robotSpeed)) {
                getPath().remove();
                follow(robotPose, robotSpeed);
            }

            Translation2d lastPointToLookaheadPoint = pathUnitVector.scale(Translation2d.dot(
                    pathUnitVector, lastPointToRobot) + nextCurvePoint.getLookaheadDistance(robotSpeed));
            Translation2d robotToLookaheadPoint = lastPointToLookaheadPoint.translateBy(lastPointToRobot.inverse());
            Translation2d powerVector = robotToLookaheadPoint.rotateBy(robotPose.getRotation());
            powerVector = powerVector.scale(1 / powerVector.norm());
            Rotation2d headingError = lastCurvePoint.getPose().getRotation().rotateBy(robotPose.getRotation().inverse());
            if(Math.abs(headingError.getRadians() + 2d * Math.PI) < Math.abs(headingError.getRadians())) {
                headingError = headingError.rotateBy(new Rotation2d(2d * Math.PI, false));
            } else if(Math.abs(headingError.getRadians() - 2d * Math.PI) < Math.abs(headingError.getRadians())) {
                headingError = headingError.rotateBy(new Rotation2d(-2d * Math.PI, false));
            }

            Rotation2d rotationPower = headingError.multiply(getkP_TURN(), false);
            return new Pose2d(powerVector, rotationPower);
        }
    }

    public LinkedList<CurvePoint> getPath() {
        return path;
    }

    public void setPath(LinkedList<CurvePoint> path) {
        this.path = path;
    }

    public static double getkP_TURN() {
        return kP_TURN;
    }

    public static double getDistanceToFinalPointThreshold() {
        return DISTANCE_TO_FINAL_POINT_THRESHOLD;
    }

    public boolean hasReachedGoal() {
        return reachedGoal;
    }

    public void setReachedGoal(boolean reachedGoal) {
        this.reachedGoal = reachedGoal;
    }
}
