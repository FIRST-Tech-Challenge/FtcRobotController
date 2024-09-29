package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the RRToPedroThreeWheelLocalizer class. This class extends the Localizer superclass and
 * is intended to adapt the old Road Runner three wheel odometry localizer to the new Pedro Pathing
 * localizer system.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/9/2024
 */
public class RRToPedroThreeWheelLocalizer extends Localizer {
    private RoadRunnerThreeWheelLocalizer localizer;
    private double totalHeading;
    private Pose startPose;
    private Pose previousPose;

    /**
     * This creates a new RRToPedroThreeWheelLocalizer from a HardwareMap. This adapts the previously
     * used Road Runner localization system to the new Pedro Pathing localization system.
     *
     * @param hardwareMap the HardwareMap
     */
    public RRToPedroThreeWheelLocalizer(HardwareMap hardwareMap) {
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        localizer = new RoadRunnerThreeWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        startPose = new Pose();
        previousPose = new Pose();
    }

    /**
     * This returns the current pose estimate as a Pose.
     *
     * @return returns the current pose estimate
     */
    @Override
    public Pose getPose() {
        Pose2d pose = localizer.getPoseEstimate();
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * This returns the current velocity estimate as a Pose.
     *
     * @return returns the current velocity estimate
     */
    @Override
    public Pose getVelocity() {
        Pose2d pose = localizer.getPoseVelocity();
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * This returns the current velocity estimate as a Vector.
     *
     * @return returns the current velocity estimate
     */
    @Override
    public Vector getVelocityVector() {
        Pose2d pose = localizer.getPoseVelocity();
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(pose.getX(), pose.getY());
        return returnVector;
    }

    /**
     * This sets the start pose. Any movement of the robot is treated as a displacement from the
     * start pose, so moving the start pose will move the current pose estimate the same amount.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        Pose oldStart = startPose;
        startPose = setStart;
        Pose startDiff = MathFunctions.subtractPoses(startPose, oldStart);
        localizer.setPoseEstimate(new Pose2d(getPose().getX() + startDiff.getX(), getPose().getY() + startDiff.getY(), getPose().getHeading() + startDiff.getHeading()));
    }

    /**
     * This sets the current pose estimate. This has no effect on the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        localizer.setPoseEstimate(new Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading()));
    }

    /**
     * This updates the total heading and previous pose estimate. Everything else is handled by the
     * Road Runner localizer on its own, but updating this tells you how far the robot has really
     * turned.
     */
    @Override
    public void update() {
        totalHeading += MathFunctions.getTurnDirection(previousPose.getHeading(), getPose().getHeading()) * MathFunctions.getSmallestAngleDifference(previousPose.getHeading(), getPose().getHeading());
        previousPose = getPose();
    }

    /**
     * This returns how far the robot has actually turned.
     *
     * @return returns the total angle turned, in degrees.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the forward multiplier of the Road Runner localizer, which converts from ticks
     * to inches. You can actually use the tuners in Pedro Pathing to find the value that everything
     * multiplied together should be. If you do use that, then do be aware that the value returned is
     * the product of the Road Runner ticks to inches and the x multiplier.
     *
     * @return returns the forward multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return RoadRunnerThreeWheelLocalizer.encoderTicksToInches(1) * RoadRunnerThreeWheelLocalizer.X_MULTIPLIER;
    }

    /**
     * This returns the lateral multiplier of the Road Runner localizer, which converts from ticks
     * to inches. You can actually use the tuners in Pedro Pathing to find the value that everything
     * multiplied together should be. If you do use that, then do be aware that the value returned is
     * the product of the Road Runner ticks to inches and the y multiplier.
     *
     * @return returns the lateral multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return RoadRunnerThreeWheelLocalizer.encoderTicksToInches(1) * RoadRunnerThreeWheelLocalizer.Y_MULTIPLIER;
    }

    /**
     * This returns the turning multiplier of the Road Runner localizer, which doesn't actually exist.
     * There really isn't a point in tuning the turning for the Road Runner localizer. This will
     * actually just return the average of the two other multipliers.
     *
     * @return returns the turning multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return (getForwardMultiplier() + getLateralMultiplier()) / 2;
    }
}
