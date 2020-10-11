package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

import java.io.Serializable;

public class TrajectoryMotionSegment implements Serializable {
    private static final long serialVersionUID = 1L ;
    public double xDisplacement;
    public double yDisplacement;
    public double xVelocity;
    public double yVelocity;
    public double xAcceleration;
    public double yAcceleration;
    private double m_PreferredAngle;
    public double duration;
    public TrajectoryMotionSegment(double xDisplacement, double yDisplacement, double xVelocity, double yVelocity, double xAcceleration, double yAcceleration, double preferredAngle, double duration){
        this.xDisplacement = xDisplacement;
        this.yDisplacement = yDisplacement;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.xAcceleration = xAcceleration;
        this.yAcceleration = yAcceleration;
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
        this.duration = duration;
    }
    public TrajectoryMotionSegment(TrajectoryMotionSegment oldMotionState){
        this.xDisplacement = oldMotionState.xDisplacement;
        this.yDisplacement = oldMotionState.yDisplacement;
        this.xVelocity = oldMotionState.xVelocity;
        this.yVelocity = oldMotionState.yVelocity;
        this.xAcceleration = oldMotionState.xAcceleration;
        this.yAcceleration = oldMotionState.yAcceleration;
        this.m_PreferredAngle = oldMotionState.m_PreferredAngle;
        this.duration = oldMotionState.duration;
    }
    public TrajectoryMotionState getStatusAt(double time){
        return new TrajectoryMotionState(
                this.xDisplacement + this.xVelocity * time + this.xAcceleration * time * time / 2.0,
                this.yDisplacement + this.yVelocity * time + this.yAcceleration * time * time / 2.0,
                this.xVelocity + this.xAcceleration * time,
                this.yVelocity + this.yAcceleration * time,
                this.m_PreferredAngle
        );
    }
    public double getPreferredAngle(){
        return this.m_PreferredAngle;
    }
    public void setPreferredAngle(double preferredAngle){
        this.m_PreferredAngle = preferredAngle;
    }
}
