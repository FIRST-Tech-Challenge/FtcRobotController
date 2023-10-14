package org.firstinspires.ftc.teamcode.lib.util;

public class Time implements Comparable<Time> {
    private double    timeValue;
    private TimeUnits units;

    public Time(final double timeValue, final TimeUnits units) {
        setTimeValue(timeValue);
        setUnits(units);
    }

    public double getTimeValue(TimeUnits units) {
        return getUnits().in(units, getTimeValue());
    }

    public double getTimeValue() {
        return timeValue;
    }

    public Time subtract(Time time) {
        return new Time(getTimeValue() - time.getTimeValue(getUnits()), getUnits());
    }

    public void setTimeValue(double timeValue) {
        this.timeValue = timeValue;
    }

    public TimeUnits getUnits() {
        return units;
    }

    public void setUnits(TimeUnits units) {
        this.units = units;
    }

    @Override
    public int compareTo(Time another) {
        return (int)(getTimeValue(TimeUnits.NANOSECONDS) - another.getTimeValue(TimeUnits.NANOSECONDS));
    }
}
