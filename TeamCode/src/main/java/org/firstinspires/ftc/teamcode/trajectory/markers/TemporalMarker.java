package org.firstinspires.ftc.teamcode.trajectory.markers;

public abstract class TemporalMarker extends Marker{
    private double time;

    public TemporalMarker(double time) {
        this.time = time;
    }

    @Override
    public double getTimeStamp() {
        return time;
    }

    @Override
    public void createTimeStamp(double[] times) {
        // Does nothing since the time stamp already exists
    }
}
