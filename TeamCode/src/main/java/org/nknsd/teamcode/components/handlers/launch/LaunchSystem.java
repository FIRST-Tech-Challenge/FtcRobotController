package org.nknsd.teamcode.components.handlers.launch;

import org.nknsd.teamcode.components.utility.Interpolater;


public class LaunchSystem {

    private TrajectoryHandler trajectoryHandler;
    private LauncherHandler launcherHandler;

    final private Interpolater speedInterpolater;
    final private Interpolater angleInterpolater;

    final int confidence;
    final double minDistance;
    final double maxDistance;

    public LaunchSystem(Interpolater speedInterpolater, Interpolater angleInterpolater, int confidence, double minDistance, double maxDistance) {
        this.speedInterpolater = speedInterpolater;
        this.angleInterpolater = angleInterpolater;
        this.confidence = confidence;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
    }

    public void setDistance(double distance) {
        if(distance>maxDistance){
            distance = maxDistance;
        } else if(distance < minDistance){
            distance = minDistance;
        }
//        Uses interpolaters to set launch speed and launch angle
        launcherHandler.setTargetTps(speedInterpolater.getValue(distance));
        trajectoryHandler.setAngle(angleInterpolater.getValue(distance));
//        RobotLog.v("ai: " + angleInterpolater.getValue(distance));
    }

    public boolean isReady() {
//        RobotLog.v("checking confidence");
        return launcherHandler.launchConfidence() >= confidence;
    }

    public void resetConfidence(){
        launcherHandler.resetConfidence();
    }

    public void link(TrajectoryHandler trajectoryHandler, LauncherHandler launcherHandler) {
        this.trajectoryHandler = trajectoryHandler;
        this.launcherHandler = launcherHandler;
    }
}
