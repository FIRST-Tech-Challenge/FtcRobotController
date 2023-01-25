package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.subsystems.swerveDriveBase;

public class trajectoryMarker {
    private double startTime;
    private final markerType markerType;
    private final markerAction markerAction;
    private int referenceIndex;

    public trajectoryMarker(double startTime, pose2D pose2D, double turnPower){
        this.startTime = startTime;
        markerType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerType.POSITION;
        this.markerAction = () -> {
            swerveDriveBase.targetPose2D = pose2D;
            swerveDriveBase.turnPower = turnPower;
        };
    }

    /**
     * The user generated action marker, sets it time referentially according to where its added to the pool.
     * @param referenceIndex the index of the trajectorySegment marker that this action offsets from
     * @param startTime initially, this should hold the offset from its reference segment, and then on .build() will be overridden and then sorted into the correct space and order
     * @param markerAction the user specified instruction for the robot, this should be a lambda expression or the :: operator, or an @override of the interface method
     */
    public trajectoryMarker(int referenceIndex, double startTime, markerAction markerAction){
        this.referenceIndex = referenceIndex;
        this.startTime = startTime; //temporarily stores an index number before building
        markerType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerType.ACTION;
        this.markerAction = markerAction;
    }

    /**
     * The user generated action marker, sets it time based off an offset from the start of the pool.
     * @param startTime
     * @param markerAction
     */
    public trajectoryMarker(double startTime, markerAction markerAction){
        this.startTime = startTime;
        markerType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerType.ACTION;
        this.markerAction = markerAction;
    }

    public trajectoryMarker(double startTime, double targetVelocity){
        this.startTime = startTime;
        this.markerType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerType.VELOCITY;
        this.markerAction = () -> {
            swerveDriveBase.targetVelocity = targetVelocity;
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

    public org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerType getMarkerType() {
        return markerType;
    }

    public org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.markerAction getMarkerAction() {
        return markerAction;
    }

    public int getReferenceIndex() {
        return referenceIndex;
    }
}