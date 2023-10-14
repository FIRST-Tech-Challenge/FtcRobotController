package org.firstinspires.ftc.teamcode.lib.motion;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class TrapezoidalMotionProfileGenerator implements IMotionProfile {
    private static final double dt = 0.01d;
    private int timeSteps;
    private double[] positions;
    private double[] velocities;
    private double[] accelerations;
    private TimeProfiler timeProfiler;

    private double target;
    private double initialPosition;
    private double initialVelocity;
    private double maxSpeed;
    private double maxAcceleration;

    private double accelerationTime;
    private double cruisingTime;
    private double decelerationTime;
    private double duration;

    private boolean isNegative;

    public TrapezoidalMotionProfileGenerator(double target, double initialPosition, double initialVelocity,
                                             double maxSpeed, double maxAcceleration) {
        setNegative((target - initialPosition) < 0d);
        setTarget(Math.abs(target - initialPosition));
        setMaxSpeed(maxSpeed);
        setMaxAcceleration(maxAcceleration);
        setInitialPosition(initialPosition);
        setInitialVelocity(initialVelocity);
        setTimeProfiler(new TimeProfiler(false));
        setCruisingTime((getTarget() - (getMaxSpeed() * getMaxSpeed() - getInitialVelocity() * getInitialVelocity()) /
                (2 * getMaxAcceleration()) - getMaxSpeed() * getMaxSpeed() / (2 * getMaxAcceleration())) / getMaxSpeed());
        setPhaseTimes();
        setDuration(getAccelerationTime() + getCruisingTime() + getDecelerationTime());
        setTimeSteps((int)(Math.ceil(getDuration() / dt())));
        setPositions(new double[getTimeSteps()]);
        setVelocities(new double[getTimeSteps()]);
        setAccelerations(new double[getTimeSteps()]);
        generateProfile();
    }

    public void initializePhaseTimes() {
        setAccelerationTime((getMaxSpeed() - getInitialVelocity()) / getMaxAcceleration());
        setDecelerationTime(getMaxSpeed() / getMaxAcceleration());
    }

    public void setPhaseTimes() {
        initializePhaseTimes();
        if(getCruisingTime() < 0) {
            setCruisingTime(0d);
            setMaxSpeed(Math.sqrt(getMaxAcceleration() * getTarget() + getInitialVelocity() * getInitialVelocity() / 2));
            setPhaseTimes();
        }
    }

    @Override
    public void start() {
        getTimeProfiler().start();
    }

    @Override
    public void generateProfile() {
        for(int i = 0; i < getTimeSteps(); i++) {
            double timeStamp = i * dt();
            if(timeStamp < getAccelerationTime()) {
                getPositions()[i] = (getInitialVelocity() + getMaxAcceleration() * timeStamp / 2) * timeStamp;
                getVelocities()[i] = getInitialVelocity() + getMaxAcceleration() * timeStamp;
                getAccelerations()[i] = getMaxAcceleration();
            } else if(timeStamp < getAccelerationTime() + getCruisingTime()) {
                getPositions()[i] = (getMaxSpeed() + getInitialVelocity()) * getAccelerationTime() / 2
                        + getMaxSpeed() * (timeStamp - getAccelerationTime());
                getVelocities()[i] = getMaxSpeed();
                getAccelerations()[i] = 0d;
            } else if(timeStamp < getAccelerationTime() + getCruisingTime() + getDecelerationTime()) {
                getPositions()[i] = (getMaxSpeed() + getInitialVelocity()) * getAccelerationTime() / 2
                        + getMaxSpeed() * getCruisingTime() - (getMaxAcceleration() * (timeStamp - getAccelerationTime() -
                        getCruisingTime())) * (timeStamp - getAccelerationTime() - getCruisingTime()) / 2;
                getVelocities()[i] = -getMaxAcceleration() * (timeStamp - getDuration());
                getAccelerations()[i] = -getMaxAcceleration();
            } else {
                getPositions()[i] = getTarget();
                getVelocities()[i] = 0d;
                getAccelerations()[i] = 0d;
            }
        }
    }

    @Override
    public double getDuration() {
        return getDuration();
    }

    @Override
    public double getPosition() {
        return getPosition(getRuntime());
    }

    @Override
    public double getVelocity() {
        return getVelocity(getRuntime());
    }

    @Override
    public double getAcceleration() {
        return getAcceleration(getRuntime());
    }

    @Override
    public double getJerk() {
        return getJerk(getRuntime());
    }

    @Override
    public double getPosition(double timeStamp) {
        timeStamp = timeStamp < 0d ? 0d : timeStamp;
        int timeStep = (int)(timeStamp / dt());
        if(timeStep < getPositions().length) {
            return (isNegative() ? -1 : 1) * getPositions()[timeStep];
        }

        return (isNegative() ? -1 : 1) * getTarget();
    }

    @Override
    public double getVelocity(double timeStamp) {
        timeStamp = timeStamp < 0d ? 0d : timeStamp;
        int timeStep = (int)(timeStamp / dt());
        if(timeStep < getVelocities().length) {
            return (isNegative() ? -1 : 1) * getVelocities()[timeStep];
        }

        return 0d;
    }

    @Override
    public double getAcceleration(double timeStamp) {
        timeStamp = timeStamp < 0d ? 0d : timeStamp;
        int timeStep = (int)(timeStamp / dt());
        if(timeStep < getAccelerations().length) {
            return (isNegative() ? -1 : 1) * getAccelerations()[timeStep];
        }

        return 0d;
    }

    @Override
    public double getJerk(double timeStamp) {
        return 0d;
    }

    @Override
    public double getRuntime() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false);
    }

    @Override
    public boolean isDone(double timeStamp) {
        return timeStamp >= getDuration();
    }

    @Override
    public boolean isDone() {
        return isDone(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    public double[] getPositions() {
        return positions;
    }

    public void setPositions(double[] positions) {
        this.positions = positions;
    }

    public double[] getVelocities() {
        return velocities;
    }

    public void setVelocities(double[] velocities) {
        this.velocities = velocities;
    }

    public double[] getAccelerations() {
        return accelerations;
    }

    public void setAccelerations(double[] accelerations) {
        this.accelerations = accelerations;
    }

    public int getTimeSteps() {
        return timeSteps;
    }

    public void setTimeSteps(int timeSteps) {
        this.timeSteps = timeSteps;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getInitialPosition() {
        return initialPosition;
    }

    public void setInitialPosition(double initialPosition) {
        this.initialPosition = initialPosition;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public double getInitialVelocity() {
        return initialVelocity;
    }

    public void setInitialVelocity(double initialVelocity) {
        this.initialVelocity = initialVelocity;
    }

    public double getAccelerationTime() {
        return accelerationTime;
    }

    public void setAccelerationTime(double accelerationTime) {
        this.accelerationTime = accelerationTime;
    }

    public double getCruisingTime() {
        return cruisingTime;
    }

    public void setCruisingTime(double cruisingTime) {
        this.cruisingTime = cruisingTime;
    }

    public double getDecelerationTime() {
        return decelerationTime;
    }

    public void setDecelerationTime(double decelerationTime) {
        this.decelerationTime = decelerationTime;
    }

    public void setDuration(double duration) {
        this.duration = duration;
    }

    @Override
    public boolean isNegative() {
        return isNegative;
    }

    @Override
    public void setNegative(boolean negative) {
        isNegative = negative;
    }

    public static double dt() {
        return dt;
    }
}
