package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;

import java.util.NoSuchElementException;

public class MotionProfileIterator implements RobotMotionProfilingIterator<MotionState,MotionProfileSegment> {
    private MotionProfile m_Profile;
    private double m_CurrentDuration;
    private MotionState m_CurrentState;
    private int m_CurrentIndex;
    private double m_CurrentSegmentDuration;
    private double m_TotalDuration;

    public MotionProfileIterator(MotionProfile profile){
        this.m_Profile = profile;
        this.m_CurrentDuration = 0;
        this.m_CurrentState = profile.getMotionStateAt(0);
        this.m_CurrentIndex = 0;
        this.m_CurrentSegmentDuration = 0;
        this.m_TotalDuration = profile.getTotalDuration();
    }

    public MotionProfileIterator(MotionProfileIterator oldIterator){
        this.m_Profile = oldIterator.m_Profile;
        this.m_CurrentDuration = oldIterator.m_CurrentDuration;
        this.m_CurrentState = new MotionState(oldIterator.m_CurrentState);
        this.m_CurrentIndex = oldIterator.m_CurrentIndex;
        this.m_CurrentSegmentDuration = oldIterator.m_CurrentSegmentDuration;
        this.m_TotalDuration = oldIterator.m_TotalDuration;
    }

    @Override
    public MotionState forward(double deltaDuration) throws NoSuchElementException {
        if(deltaDuration < 0){
            return this.backward(-deltaDuration);
        }
        double targetDuration = this.m_CurrentDuration + deltaDuration;
        if(targetDuration > this.m_TotalDuration){
            throw new NoSuchElementException();
        }
        MotionProfileSegment segmentGettingCounted;

        boolean firstSegment = true;
        boolean finalSegment = false;

        double durationCounter = m_CurrentDuration;
        double distanceCounter = this.m_CurrentState.distance;
        double speedCounter = this.m_CurrentState.velocity;

        for(int i = this.m_CurrentIndex; i < this.m_Profile.getMotionSegments().size(); i++){
            if(firstSegment){
                segmentGettingCounted = this.m_Profile.getMotionSegments().get(i);
                segmentGettingCounted = segmentGettingCounted.clipped(this.m_CurrentSegmentDuration,segmentGettingCounted.getDuration() - this.m_CurrentSegmentDuration);
                firstSegment = false;
            }else{
                segmentGettingCounted = this.m_Profile.getMotionSegments().get(i);
            }
            if(durationCounter + segmentGettingCounted.getDuration() >= targetDuration){
                finalSegment = true;
                segmentGettingCounted = segmentGettingCounted.clipped(0,targetDuration - durationCounter);
            }
            durationCounter += segmentGettingCounted.getDuration();
            distanceCounter += segmentGettingCounted.getDistanceTravelled(speedCounter,segmentGettingCounted.getDuration());
            speedCounter += segmentGettingCounted.getTotalDeltaSpeed();
            if(finalSegment){
                this.m_CurrentSegmentDuration = segmentGettingCounted.getDuration();
                this.m_CurrentDuration = durationCounter;
                this.m_CurrentIndex = i;
                this.m_CurrentState.distance = distanceCounter;
                this.m_CurrentState.velocity = speedCounter;
                this.m_CurrentState.acceleration = segmentGettingCounted.getAccelerationAt(segmentGettingCounted.getDuration());
                this.m_CurrentState.jerk = segmentGettingCounted.jerk;
                return this.m_CurrentState;
            }
        }
        throw new NoSuchElementException();
    }

    @Override
    public MotionState backward(double deltaBackwardDuration) throws NoSuchElementException {
        if(deltaBackwardDuration < 0){
            return this.forward(-deltaBackwardDuration);
        }
        double targetDuration = this.m_CurrentDuration - deltaBackwardDuration;
        if(targetDuration > this.m_TotalDuration){
            throw new NoSuchElementException();
        }
        MotionProfileSegment segmentGettingCounted;

        boolean firstSegment = true;
        boolean finalSegment = false;

        double durationCounter = m_CurrentDuration;
        double distanceCounter = this.m_CurrentState.distance;
        double speedCounter = this.m_CurrentState.velocity;

        double finalSegmentDuration = 0;
        for(int i = this.m_CurrentIndex; i < this.m_Profile.getMotionSegments().size(); i--){
            if(firstSegment){
                segmentGettingCounted = this.m_Profile.getMotionSegments().get(i);
                segmentGettingCounted = segmentGettingCounted.clipped(0,this.m_CurrentSegmentDuration);
                firstSegment = false;
            }else{
                segmentGettingCounted = this.m_Profile.getMotionSegments().get(i);
            }
            if(durationCounter - segmentGettingCounted.getDuration() <= targetDuration){
                finalSegment = true;
                finalSegmentDuration = segmentGettingCounted.getDuration() - (durationCounter - targetDuration);
                segmentGettingCounted = segmentGettingCounted.clipped(finalSegmentDuration,segmentGettingCounted.getDuration());
            }
            speedCounter -= segmentGettingCounted.getTotalDeltaSpeed();
            durationCounter -= segmentGettingCounted.getDuration();
            distanceCounter -= segmentGettingCounted.getDistanceTravelled(speedCounter,segmentGettingCounted.getDuration());

            if(finalSegment){
                this.m_CurrentSegmentDuration = finalSegmentDuration;
                this.m_CurrentDuration = durationCounter;
                this.m_CurrentIndex = i;
                this.m_CurrentState.distance = distanceCounter;
                this.m_CurrentState.velocity = speedCounter;
                this.m_CurrentState.acceleration = segmentGettingCounted.getAccelerationAt(0);
                this.m_CurrentState.jerk = segmentGettingCounted.jerk;
                return this.m_CurrentState;
            }
        }
        throw new NoSuchElementException();
    }

    @Override
    public MotionState current() {
        return this.m_CurrentState;
    }

    @Override
    public MotionProfileSegment currentSegment() {
        return this.m_Profile.getMotionSegments().get(this.m_CurrentIndex);
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
