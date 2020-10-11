package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.odometry.OdometryMethod;

/**
 * Standard 3 Wheel Tracker
 * The three tracking wheels are installed at the left, front and right side of the robot
 */
public abstract class Robot3Wheel2DTracker extends OdometryMethod {
    private int m_LastLeftEncoderCount, m_LastMidEncoderCount, m_LastRightEncoderCount;
    private double c_LEFTENCODER_COUNTS_PER_CM, c_MIDENCODER_COUNTS_PER_CM, c_RIGHTENCODER_COUNTS_PER_CM;
    private double c_MIDENCODER_CM_PER_DEG, c_LEFTENCODER_CM_PER_DEG, c_RIGHTENCODER_CM_PER_DEG;


    private boolean m_LeftEncoderReversed = false;
    private boolean m_MidEncoderReversed = false;
    private boolean m_RightEncoderReversed = false;

    private MotorType m_LeftEncoderMotorType, m_MidEncoderMotorType, m_RightEncoderMotorType;
    private RobotWheel m_LeftEncoderWheel, m_MidEncoderWheel, m_RightEncoderWheel;

    public Robot3Wheel2DTracker(boolean LeftEncoderReversed, MotorType LeftEncoderMotorType, RobotWheel LeftEncoderWheel, boolean MidEncoderReversed, MotorType MidEncoderMotorType, RobotWheel MidEncoderWheel, boolean RightEncoderReversed, MotorType RightEncoderMotorType, RobotWheel RightEncoderWheel) {
        this.m_LeftEncoderReversed = LeftEncoderReversed;
        this.m_LeftEncoderMotorType = LeftEncoderMotorType;
        this.m_LeftEncoderWheel = LeftEncoderWheel;
        this.m_MidEncoderReversed = MidEncoderReversed;
        this.m_MidEncoderMotorType = MidEncoderMotorType;
        this.m_MidEncoderWheel = MidEncoderWheel;
        this.m_RightEncoderReversed = RightEncoderReversed;
        this.m_RightEncoderMotorType = RightEncoderMotorType;
        this.m_RightEncoderWheel = RightEncoderWheel;
    }

    public Robot3Wheel2DTracker(Robot3Wheel2DTracker oldTracker) {
        super(oldTracker);
        this.m_LeftEncoderReversed = oldTracker.m_LeftEncoderReversed;
        this.m_LeftEncoderMotorType = oldTracker.m_LeftEncoderMotorType;
        this.m_LeftEncoderWheel = oldTracker.m_LeftEncoderWheel;
        this.m_MidEncoderReversed = oldTracker.m_MidEncoderReversed;
        this.m_MidEncoderMotorType = oldTracker.m_MidEncoderMotorType;
        this.m_MidEncoderWheel = oldTracker.m_MidEncoderWheel;
        this.m_RightEncoderReversed = oldTracker.m_RightEncoderReversed;
        this.m_RightEncoderMotorType = oldTracker.m_RightEncoderMotorType;
        this.m_RightEncoderWheel = oldTracker.m_RightEncoderWheel;
    }


    protected abstract void updateData();
    protected abstract int getLeftEncoderCount();
    protected abstract int getMidEncoderCount();
    protected abstract int getRightEncoderCount();

    @Override
    public void __trackStart() {
        c_LEFTENCODER_COUNTS_PER_CM = m_LeftEncoderMotorType.getCountsPerRev() / m_LeftEncoderWheel.getCircumference();
        c_MIDENCODER_COUNTS_PER_CM = m_MidEncoderMotorType.getCountsPerRev() / m_MidEncoderWheel.getCircumference();
        c_RIGHTENCODER_COUNTS_PER_CM = m_RightEncoderMotorType.getCountsPerRev() / m_RightEncoderWheel.getCircumference();
        c_LEFTENCODER_CM_PER_DEG = m_LeftEncoderWheel.getDistanceFromCenterOfRobot() * 2.0 * Math.PI / 360.0;
        c_MIDENCODER_CM_PER_DEG = m_MidEncoderWheel.getDistanceFromCenterOfRobot() * 2.0 * Math.PI / 360.0;
        c_RIGHTENCODER_CM_PER_DEG = m_RightEncoderWheel.getDistanceFromCenterOfRobot() * 2.0 * Math.PI / 360.0;

        this.updateData();

        m_LastLeftEncoderCount = this.getLeftEncoderCount();
        m_LastMidEncoderCount = this.getMidEncoderCount();
        m_LastRightEncoderCount = this.getRightEncoderCount();
    }

    @Override
    public void __trackLoop(double secondsSinceLastLoop) {
        this.updateData();
        int newMidCount = this.getMidEncoderCount();
        int newLeftCount = this.getLeftEncoderCount();
        int newRightCount = this.getRightEncoderCount();

        int deltaMidCount = newMidCount - m_LastMidEncoderCount;
        int deltaLeftCount = newLeftCount - m_LastLeftEncoderCount;
        int deltaRightCount = newRightCount - m_LastRightEncoderCount;

        if(m_MidEncoderReversed)
            deltaMidCount = -deltaMidCount;
        if(m_LeftEncoderReversed)
            deltaLeftCount = -deltaLeftCount;
        if(m_RightEncoderReversed)
            deltaRightCount = -deltaRightCount;

        double deltaMidCM = deltaMidCount / c_MIDENCODER_COUNTS_PER_CM;
        double deltaLeftCM = deltaLeftCount / c_LEFTENCODER_COUNTS_PER_CM;
        double deltaRightCM = deltaRightCount / c_RIGHTENCODER_COUNTS_PER_CM;

        double deltaAngMoved = (deltaLeftCM / c_LEFTENCODER_CM_PER_DEG + deltaRightCM / c_RIGHTENCODER_CM_PER_DEG) / 2;

        double deltaXMoved = (-(deltaLeftCM - deltaAngMoved * c_LEFTENCODER_CM_PER_DEG) + (deltaRightCM - deltaAngMoved * c_RIGHTENCODER_CM_PER_DEG)) / 2.0;
        double deltaYMoved = deltaMidCM - deltaAngMoved * c_MIDENCODER_CM_PER_DEG;

        deltaAngMoved /= this.getPositionTracker().getRotZDistanceFactor();
        deltaAngMoved = XYPlaneCalculations.normalizeDeg(deltaAngMoved);
        deltaAngMoved = this.getPositionTracker().__getDeltaAng(deltaAngMoved);

        deltaXMoved /= this.getPositionTracker().getXDistanceFactor();
        deltaYMoved /= this.getPositionTracker().getYDistanceFactor();

        RobotPose2D currentVelocityVector = new RobotPose2D(
                deltaXMoved / secondsSinceLastLoop,
                deltaYMoved / secondsSinceLastLoop,
                deltaAngMoved / secondsSinceLastLoop
        );

        this.getPositionTracker().__trackLoopMoved(currentVelocityVector,
                new RobotPose2D(
                    deltaXMoved,
                    deltaYMoved,
                    deltaAngMoved
                )
        );

        m_LastMidEncoderCount = newMidCount;
        m_LastLeftEncoderCount = newLeftCount;
        m_LastRightEncoderCount = newRightCount;
    }
    public void __trackLoop_NoActualPositionShift(double secondsSinceLastLoop){
        this.updateData();
        int newMidCount = this.getMidEncoderCount();
        int newLeftCount = this.getLeftEncoderCount();
        int newRightCount = this.getRightEncoderCount();
        m_LastMidEncoderCount = newMidCount;
        m_LastLeftEncoderCount = newLeftCount;
        m_LastRightEncoderCount = newRightCount;
    }
}
