package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

public class MotionProfileSegment {
    public double acceleration;
    public double jerk;
    private double m_Duration;
    public MotionProfileSegment(double acceleration, double jerk, double duration){
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.m_Duration = Math.abs(duration);
    }
    public MotionProfileSegment(MotionProfileSegment oldSegment){
        this.acceleration = oldSegment.acceleration;
        this.jerk = oldSegment.jerk;
        this.m_Duration = oldSegment.m_Duration;
    }
    public double getDuration(){
        return this.m_Duration;
    }
    public void setDuration(double duration){
        this.m_Duration = Math.abs(duration);
    }
    public double getTotalDeltaSpeed(){
        return this.getTotalDeltaSpeed(this.m_Duration);
    }
    public double getTotalDeltaSpeed(double duration){
        return this.acceleration * duration + this.jerk * Math.pow(duration,2) / 2.0;
    }
    public double getAccelerationAt(double duration){
        return this.acceleration + this.jerk * duration;
    }
    public double getAccelerationBetween(double startDuration, double endDuration){
        return this.acceleration + this.jerk * (endDuration + startDuration) / 2.0;
    }
    public double getDistanceTravelled(double initialSpeed, double duration){
        return initialSpeed * duration + (this.acceleration * Math.pow(duration,2) / 2.0 + this.jerk * Math.pow(duration,3) / 6.0);
    }
    public MotionProfileSegment reversed(){
        double endAccel = this.getAccelerationAt(this.getDuration());
        return new MotionProfileSegment(endAccel,-jerk,m_Duration);
    }
    public MotionProfileSegment clipped(double startDuration, double endDuration){
        return new MotionProfileSegment(this.getAccelerationAt(startDuration),this.jerk,endDuration - startDuration);
    }
    public MotionProfileSegment negative(){
        return new MotionProfileSegment(-acceleration,-jerk,m_Duration);
    }
}
