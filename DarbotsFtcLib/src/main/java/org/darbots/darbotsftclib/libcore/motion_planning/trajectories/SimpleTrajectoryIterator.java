package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;

import java.util.NoSuchElementException;

public class SimpleTrajectoryIterator implements RobotMotionProfilingIterator<TrajectoryMotionState,TrajectoryMotionSegment> {
    private SimpleTrajectory m_Trajectory;
    private double m_CurrentDuration;
    private double m_TotalDuration;
    private int m_CurrentIndex;
    private TrajectoryMotionState m_CurrentState;

    public SimpleTrajectoryIterator(SimpleTrajectory trajectory){
        this.m_Trajectory = trajectory;
        this.m_CurrentDuration = 0;
        this.m_CurrentIndex = 0;
        this.m_TotalDuration = trajectory.getTotalDuration();
        this.m_CurrentState = this.getStateAt(0);
    }

    public TrajectoryMotionState getStateAt(double targetDuration) throws NoSuchElementException{
        if(targetDuration < 0 || targetDuration > this.m_TotalDuration){
            throw new NoSuchElementException();
        }
        double indexTempDouble = targetDuration / this.m_Trajectory.getResolution();
        int indexInTrajectorySegments = (int) Math.floor(indexTempDouble);
        double durationInSegment = targetDuration - indexInTrajectorySegments * this.m_Trajectory.getResolution();
        TrajectoryMotionSegment currentSegment = this.m_Trajectory.getMotionSegments().get(indexInTrajectorySegments);

        this.m_CurrentDuration = targetDuration;
        this.m_CurrentIndex = indexInTrajectorySegments;
        this.m_CurrentState = currentSegment.getStatusAt(durationInSegment);
        return this.m_CurrentState;
    }

    @Override
    public TrajectoryMotionState forward(double deltaDuration) throws NoSuchElementException {
        return this.getStateAt(this.m_CurrentDuration + deltaDuration);
    }

    @Override
    public TrajectoryMotionState backward(double deltaBackwardDuration) throws NoSuchElementException {
        return this.getStateAt(this.m_CurrentDuration - deltaBackwardDuration);
    }

    @Override
    public TrajectoryMotionState current() {
        return this.m_CurrentState;
    }

    @Override
    public TrajectoryMotionSegment currentSegment() {
        return this.m_Trajectory.getMotionSegments().get(this.m_CurrentIndex);
    }

    @Override
    public double getCurrentDuration() {
        return this.m_CurrentDuration;
    }

    @Override
    public double getTotalDuration() {
        return this.m_TotalDuration;
    }
}
