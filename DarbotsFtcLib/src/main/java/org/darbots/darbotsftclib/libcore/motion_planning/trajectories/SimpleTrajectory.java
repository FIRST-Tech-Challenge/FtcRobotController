package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;

import java.io.Serializable;
import java.util.ArrayList;

public class SimpleTrajectory implements RobotTrajectory, Serializable {
    private static final long serialVersionUID = 1L;
    private ArrayList<TrajectoryMotionSegment> m_MotionSegments;
    private double m_Resolution = 0;

    public SimpleTrajectory(double resolution){
        this.m_MotionSegments = new ArrayList<TrajectoryMotionSegment>();
        this.m_Resolution = resolution;
    }

    public SimpleTrajectory(SimpleTrajectory oldTrajectory){
        this.m_MotionSegments = new ArrayList<TrajectoryMotionSegment>(oldTrajectory.m_MotionSegments);
        this.m_Resolution = oldTrajectory.m_Resolution;
    }

    @Override
    public RobotMotionProfilingIterator<TrajectoryMotionState, TrajectoryMotionSegment> getIterator() {
        return new SimpleTrajectoryIterator(this);
    }

    @Override
    public TrajectoryMotionState getEndState() {
        if(this.m_MotionSegments.isEmpty()){
            return new TrajectoryMotionState(0,0,0,0,0);
        }else{
            TrajectoryMotionSegment lastSegment = this.m_MotionSegments.get(this.m_MotionSegments.size() - 1);
            return lastSegment.getStatusAt(lastSegment.duration);
        }
    }

    public ArrayList<TrajectoryMotionSegment> getMotionSegments(){
        return this.m_MotionSegments;
    }

    public double getTotalDuration(){
        double durationCounter = 0;
        durationCounter += this.m_Resolution * (this.m_MotionSegments.size() - 1);
        durationCounter += this.m_MotionSegments.get(this.m_MotionSegments.size() - 1).duration;
        return durationCounter;
    }

    public double getResolution(){
        return this.m_Resolution;
    }
}
