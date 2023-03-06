package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDriveBase;

public class TrajectoryMarker {
    private double startTime;
    private final MarkerType markerType;
    private final MarkerAction markerAction;

    private MarkerType referenceType;
    private int referenceIndex;

    public TrajectoryMarker(double startTime, Pose2D pose2D, double turnPower){
        this.startTime = startTime;
        markerType = MarkerType.POSITION;
        this.markerAction = () -> {
            SwerveDriveBase.targetPose2D = pose2D;
            SwerveDriveBase.turnPower = turnPower;
        };
    }

    /**
     * The user generated action marker, sets it time referentially according to where its added to the pool.
     * @param referenceIndex the index of the referenceType markerType that this action offsets from
     * @param referenceType the reference type of the trajectoryMarker to refer to
     * @param startTime initially, this should hold the offset from its reference segment, and then on .build() will be overridden and then sorted into the correct space and order
     * @param markerAction the user specified instruction for the robot, this should be a lambda expression or the :: operator, or an @override of the interface method
     */
    public TrajectoryMarker(int referenceIndex, MarkerType referenceType, double startTime, MarkerAction markerAction){
        this.referenceIndex = referenceIndex;
        this.referenceType = referenceType;
        this.startTime = startTime;
        markerType = MarkerType.ACTION;
        this.markerAction = markerAction;
    }

    /**
     * The user generated action marker, sets it time based off an offset from the start of the pool.
     * @param startTime
     * @param markerAction
     */
    public TrajectoryMarker(double startTime, MarkerAction markerAction){
        this.startTime = startTime;
        markerType = MarkerType.ACTION;
        this.markerAction = markerAction;
    }

    public TrajectoryMarker(double startTime, double targetVelocity){
        this.startTime = startTime;
        this.markerType = MarkerType.VELOCITY;
        this.markerAction = () -> {
            SwerveDriveBase.targetVelocity = targetVelocity;
        };
    }

    public double getStartTime() {
        return startTime;
    }


    /**
     * this method is used for setting the start time of the temporal offset action markers, as their start times previously stored an index number
     * @param startTime the start time of a trajectory marker, when its instruction is sent to the robot
     */
    public void setStartTime(double startTime) {
        this.startTime = startTime;
    }

    public MarkerType getMarkerType() {
        return markerType;
    }

    public MarkerAction getMarkerAction() {
        return markerAction;
    }

    public int getReferenceIndex() {
        return referenceIndex;
    }

    public MarkerType getReferenceType() {
        return referenceType;
    }
}