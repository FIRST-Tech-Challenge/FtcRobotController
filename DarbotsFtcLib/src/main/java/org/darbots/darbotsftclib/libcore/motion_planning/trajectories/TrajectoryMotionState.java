package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public class TrajectoryMotionState {
    public double xDisplacement;
    public double yDisplacement;
    public double xVelocity;
    public double yVelocity;
    private double m_PreferredAngle;
    public TrajectoryMotionState(double xDisplacement, double yDisplacement, double xVelocity, double yVelocity, double preferredAngle){
        this.setValues(xDisplacement,yDisplacement,xVelocity,yVelocity,preferredAngle);
    }
    public TrajectoryMotionState(TrajectoryMotionState oldMotionState){
        this.setValues(oldMotionState);
    }
    public double getPreferredAngle(){
        return this.m_PreferredAngle;
    }
    public void setPreferredAngle(double preferredAngle){
        this.m_PreferredAngle = preferredAngle;
    }
    public void setValues(TrajectoryMotionState motionState){
        this.xDisplacement = motionState.xDisplacement;
        this.yDisplacement = motionState.yDisplacement;
        this.xVelocity = motionState.xVelocity;
        this.yVelocity = motionState.yVelocity;
        this.m_PreferredAngle = motionState.m_PreferredAngle;
    }
    public void setValues(double xDisplacement, double yDisplacement, double xVelocity, double yVelocity, double preferredAngle){
        this.xDisplacement = xDisplacement;
        this.yDisplacement = yDisplacement;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
    }
}
