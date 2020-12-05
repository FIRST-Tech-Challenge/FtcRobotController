package org.firstinspires.ftc.teamcode.lib.motion;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.control.StackTracker;

/**
 * https://www.researchgate.net/publication/267794207_Motion_profile_planning_for_reduced_jerk_and_vibration_residuals
 *
 * https://core.ac.uk/download/pdf/143892523.pdf
 *
 * https://hal.archives-ouvertes.fr/hal-01064647/document
 *
 * http://www.dii.unimore.it/~lbiagiotti/Papers/Optimal%20Trajectories%20for%20Vibration%20Reduction%20Based%20on%20Exponential%20Filters_PostPrint
 * .pdf
 */
public class ResidualVibrationReductionMotionProfilerGenerator implements IMotionProfile {
    private static final ResidualVibrationReductionMotionProfilerGenerator[] STANDARD_MOTION_PROFILES;
    private static final ResidualVibrationReductionMotionProfilerGenerator FOUNDATION_MOTION_PROFILE;

    private static final double dt = 0.01d;
    private TimeProfiler timeProfiler;
    private double[] positions;
    private double[] velocities;
    private double[] accelerations;
    private double[] jerks;
    private double targetDisplacement;
    private double maxSpeed;
    private double maxAcceleration;
    private double accelerationTime;
    private double totalTime;
    private boolean hasConstantVelocityStep;
    private double startTime;
    private boolean isNegative;
    private double initialPosition;

    static {
        STANDARD_MOTION_PROFILES = new ResidualVibrationReductionMotionProfilerGenerator[16];
        FOUNDATION_MOTION_PROFILE = new ResidualVibrationReductionMotionProfilerGenerator(
                0d, 24d, 30d, 50d
        );
    }

    public static void init() {
        for(int i = 0; i < STANDARD_MOTION_PROFILES.length / 2; i++) {
            //STANDARD_MOTION_PROFILES[i] = new ResidualVibrationReductionMotionProfilerGenerator(0d, StackTracker.getExtensionHeight(i + 1), 20d * 6d, 200d * 6d);
            STANDARD_MOTION_PROFILES[i] = new ResidualVibrationReductionMotionProfilerGenerator(0d, StackTracker.getExtensionHeight(i + 1), 12d, 50d);
            //STANDARD_MOTION_PROFILES[i + STANDARD_MOTION_PROFILES.length / 2] = new ResidualVibrationReductionMotionProfilerGenerator(StackTracker.getExtensionHeight(i + 1), -StackTracker.getExtensionHeight(i + 1), 40d * 6d, 400d * 6d);
            STANDARD_MOTION_PROFILES[i + STANDARD_MOTION_PROFILES.length / 2] = new ResidualVibrationReductionMotionProfilerGenerator(StackTracker.getExtensionHeight(i + 1), -StackTracker.getExtensionHeight(i + 1), 50d, 75d);
        }
    }

    public ResidualVibrationReductionMotionProfilerGenerator(double initialPosition, double displacement, double maxSpeed, double maxAcceleration) {
        setInitialPosition(initialPosition);
        setNegative(displacement < 0d);
        setTimeProfiler(new TimeProfiler(false));
        setTargetDisplacement((isNegative() ? -1d : 1d) * displacement);
        setMaxSpeed(maxSpeed);
        setMaxAcceleration(maxAcceleration);
        double accelerationTime = 2 * getMaxSpeed() / getMaxAcceleration();
        double totalAcclerationDisplacements = 2 * Math.pow(getMaxSpeed(), 2) / getMaxAcceleration();
        if(totalAcclerationDisplacements <= displacement) {
            setHasConstantVelocityStep(true);
            setAccelerationTime(accelerationTime);
            setTotalTime(2 * getAccelerationTime() + getConstantVelocityTime());
        } else {
            setHasConstantVelocityStep(false);
            setAccelerationTime(Math.sqrt(2 * getTargetDisplacement() / getMaxAcceleration()));
            setTotalTime(2 * getAccelerationTime());
        }

        final int timeSteps = (int)(Math.ceil(getTotalTime() / getDt()));
        setPositions(new double[timeSteps]);
        setVelocities(new double[timeSteps]);
        setAccelerations(new double[timeSteps]);
        setJerks(new double[timeSteps]);
        generateProfile();
    }

    public static void main(String... args) {
        ResidualVibrationReductionMotionProfilerGenerator motionProfiler =
                new ResidualVibrationReductionMotionProfilerGenerator(0d, 15.66667d, 21.6d, 200d);
        System.out.println(motionProfiler);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("Time\tPosition\tVelocity\tAcceleration\tJerk\n");
        for(int i = 0; i * getDt() <= getTotalTime(); i++) {
            builder.append(i * getDt()).append('\t').append(getPosition(i * getDt())).append('\t')
                    .append(getCurrentVelocity(i * getDt())).append('\t').append(getCurrentAcceleration(i * getDt())).append('\t')
                    .append(getCurrentJerk(i * getDt())).append('\n');
        }

        return builder.toString();
    }

    @Override
    public void generateProfile() {
        for(int i = 0; i * getDt() < getTotalTime(); i++) {
            double timeStamp = i * getDt();
            getPositions()[i] = getCurrentDisplacement(timeStamp);
            getVelocities()[i] = getCurrentVelocity(timeStamp);
            getAccelerations()[i] = getCurrentAcceleration(timeStamp);
            getJerks()[i] = getCurrentJerk(timeStamp);
        }
    }

    @Override
    public double getDuration() {
        return getTotalTime();
    }

    @Override
    public double getPosition() {
        return getPosition(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    @Override
    public double getVelocity() {
        return getVelocity(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    @Override
    public double getAcceleration() {
        return getAcceleration(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    @Override
    public double getJerk() {
        return getJerk(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    @Override
    public double getPosition(double timeStamp) {
        return getCurrentDisplacement(timeStamp) + getInitialPosition();
    }

    @Override
    public double getVelocity(double timeStamp) {
        return getCurrentVelocity(timeStamp);
    }

    @Override
    public double getAcceleration(double timeStamp) {
        return getCurrentAcceleration(timeStamp);
    }

    @Override
    public double getJerk(double timeStamp) {
        return getCurrentJerk(timeStamp);
    }

    @Override
    public double getRuntime() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false);
    }

    @Override
    public void start() {
        getTimeProfiler().start();
    }

    private double getCurrentDisplacement(final double timeStamp) {
        double constantFactor = getConstantFactor();
        double cosinoidFrequency = 2 * Math.PI / getAccelerationTime();
        Phase currentPhase = getCurrentPhase(timeStamp);
        return (isNegative() ? -1d : 1d) * (currentPhase.ordinal() == Phase.ACCELERATE.ordinal() ? constantFactor / (2 * Math.pow(cosinoidFrequency, 2)) *
                (Math.pow(cosinoidFrequency * timeStamp, 2) / 2 - (1 - Math
                        .cos(cosinoidFrequency * timeStamp))) :
                currentPhase.ordinal() == Phase.CRUISE.ordinal() ? constantFactor * Math.pow(getAccelerationTime(), 2) / 4 +
                        constantFactor * getAccelerationTime() * (timeStamp - getAccelerationTime()) / 2 :
                        currentPhase.ordinal() == Phase.DECELERATE.ordinal() ? constantFactor * Math.pow(getAccelerationTime(), 2) / 4 +
                                constantFactor * getAccelerationTime() * getConstantVelocityTime() / 2 +
                                constantFactor / (2 * Math.pow(cosinoidFrequency, 2)) *
                                        (Math.pow(2 * Math.PI, 2) * (timeStamp - getStartDecelerationTime()) / getAccelerationTime() -
                                                Math.pow(2 * Math.PI * (timeStamp - getStartDecelerationTime()) / getAccelerationTime(), 2) / 2 +
                                                (1 - Math.cos(cosinoidFrequency * (timeStamp - getStartDecelerationTime())))) : getTargetDisplacement());
    }

    private double getCurrentVelocity(final double timeStamp) {
        double constantFactor = getConstantFactor();
        double cosinoidFrequency = 2 * Math.PI / getAccelerationTime();
        Phase currentPhase = getCurrentPhase(timeStamp);
        return (isNegative() ? -1d : 1d) * (currentPhase.ordinal() == Phase.ACCELERATE.ordinal() ?
                constantFactor / (2 * cosinoidFrequency) * (cosinoidFrequency * timeStamp - Math.sin(cosinoidFrequency * timeStamp)) :
                currentPhase.ordinal() == Phase.CRUISE.ordinal() ? constantFactor * getAccelerationTime() / 2 :
                        currentPhase.ordinal() == Phase.DECELERATE.ordinal() ? constantFactor / (2 * cosinoidFrequency) *
                                (cosinoidFrequency * (getTotalTime() - timeStamp) +
                                        Math.sin(cosinoidFrequency * (timeStamp - getStartDecelerationTime()))) : 0d);
    }

    private double getCurrentAcceleration(final double timeStamp) {
        double constantFactor = getConstantFactor();
        double cosinoidFrequency = 2 * Math.PI / getAccelerationTime();
        Phase currentPhase = getCurrentPhase(timeStamp);
        return (isNegative() ? -1d : 1d) * (currentPhase.ordinal() == Phase.ACCELERATE.ordinal() ? constantFactor / 2 * (1 - Math.cos(cosinoidFrequency * timeStamp)) :
                currentPhase.ordinal() == Phase.CRUISE.ordinal() ? 0d : currentPhase.ordinal() == Phase.DECELERATE.ordinal() ?
                        -constantFactor / 2 * (1 - Math.cos(cosinoidFrequency * (timeStamp - getStartDecelerationTime()))) : 0d);
    }

    private double getCurrentJerk(final double timeStamp) {
        double constantFactor = getConstantFactor();
        double cosinoidFrequency = 2 * Math.PI / getAccelerationTime();
        Phase currentPhase = getCurrentPhase(timeStamp);
        return (isNegative() ? -1d : 1d) * (currentPhase.ordinal() == Phase.ACCELERATE.ordinal() ?
                constantFactor * Math.PI / getAccelerationTime() * Math.sin(cosinoidFrequency * timeStamp) :
                currentPhase.ordinal() == Phase.CRUISE.ordinal() ? 0d : currentPhase.ordinal() == Phase.DECELERATE.ordinal() ?
                        -constantFactor * Math.PI / getAccelerationTime() * Math
                                .sin(cosinoidFrequency * (timeStamp - getStartDecelerationTime())) : 0d);
    }

    @Override
    public boolean isDone(double timeStamp) {
        return timeStamp >= getTotalTime();
    }

    public boolean isDone() {
        return isDone(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false));
    }

    public Phase getCurrentPhase(double time) {
        return time < getAccelerationTime() ? Phase.ACCELERATE : time < getAccelerationTime() + getConstantVelocityTime() ? Phase.CRUISE :
                time < 2 * getAccelerationTime() + getConstantVelocityTime() ? Phase.DECELERATE : Phase.DONE;
    }

    protected double getConstantFactor() {
        return 2 * getTargetDisplacement() / (Math.pow(getAccelerationTime(), 2) + getAccelerationTime() * getConstantVelocityTime());
    }

    public double getStartDecelerationTime() {
        return getAccelerationTime() + getConstantVelocityTime();
    }

    public double getConstantVelocityTime() {
        return hasConstantVelocityStep() ?
                2 * (getTargetDisplacement() - 2 * Math.pow(getMaxSpeed(), 2) / getMaxAcceleration()) / (getMaxAcceleration() * getAccelerationTime()) :
                0d;
    }
    
    public static double getDt() {
        return dt;
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

    public double[] getJerks() {
        return jerks;
    }

    public void setJerks(double[] jerks) {
        this.jerks = jerks;
    }

    private double getTargetDisplacement() {
        return targetDisplacement;
    }

    private void setTargetDisplacement(double targetDisplacement) {
        this.targetDisplacement = targetDisplacement;
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

    public double getAccelerationTime() {
        return accelerationTime;
    }

    public void setAccelerationTime(double accelerationTime) {
        this.accelerationTime = accelerationTime;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public void setTotalTime(double totalTime) {
        this.totalTime = totalTime;
    }

    public boolean hasConstantVelocityStep() {
        return hasConstantVelocityStep;
    }

    public void setHasConstantVelocityStep(boolean hasConstantVelocityStep) {
        this.hasConstantVelocityStep = hasConstantVelocityStep;
    }

    public double getStartTime() {
        return startTime;
    }

    public void setStartTime(double startTime) {
        this.startTime = startTime;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    @Override
    public boolean isNegative() {
        return isNegative;
    }

    @Override
    public void setNegative(boolean negative) {
        isNegative = negative;
    }

    public double getInitialPosition() {
        return initialPosition;
    }

    public void setInitialPosition(double initialPosition) {
        this.initialPosition = initialPosition;
    }

    enum Phase {
        ACCELERATE,
        CRUISE,
        DECELERATE,
        DONE;

        public String toString() {
            return ordinal() == ACCELERATE.ordinal() ? "Speeding Up" :
                    ordinal() == CRUISE.ordinal() ? "Cruising" : ordinal() == DECELERATE.ordinal() ? "Slowing Down" : "Finished";
        }
    }

    public static ResidualVibrationReductionMotionProfilerGenerator[] getStandardMotionProfiles() {
        return STANDARD_MOTION_PROFILES;
    }

    public static ResidualVibrationReductionMotionProfilerGenerator getFoundationMotionProfile() {
        return FOUNDATION_MOTION_PROFILE;
    }
}
