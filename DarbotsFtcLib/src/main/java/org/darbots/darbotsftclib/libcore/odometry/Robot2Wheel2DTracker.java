package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.odometry.OdometryMethod;


public abstract class Robot2Wheel2DTracker extends OdometryMethod {
    private int m_LastEncoder1Count = 0;
    private int m_LastEncoder2Count = 0;
    private float m_LastGyroReading = 0.0f;

    private double c_ENCODER1_COUNTS_PER_CM, c_ENCODER2_COUNTS_PER_CM;
    private double c_ENCODER1_CM_PER_DEG, c_ENCODER2_CM_PER_DEG;
    private double c_ENCODER1_THETA_ANG, c_ENCODER2_THETA_ANG;
    private double c_ENCODER1_BETA_ANG, c_ENCODER2_BETA_ANG;
    private double c_TRANSFORMATION_MATRIX[][] = {{0,0},{0,0}};

    private boolean m_Encoder1Reversed = false;
    private boolean m_Encoder2Reversed = false;

    private MotorType m_Encoder1MotorType, m_Encoder2MotorType;
    private RobotWheel m_Encoder1Wheel, m_Encoder2Wheel;

    public Robot2Wheel2DTracker(boolean Encoder1Reversed, MotorType Encoder1MotorType, RobotWheel Encoder1Wheel, boolean Encoder2Reversed, MotorType Encoder2MotorType, RobotWheel Encoder2Wheel) {
        super();
        this.m_Encoder1Reversed = Encoder1Reversed;
        this.m_Encoder1MotorType = Encoder1MotorType;
        this.m_Encoder1Wheel = Encoder1Wheel;
        this.m_Encoder2Reversed = Encoder2Reversed;
        this.m_Encoder2MotorType = Encoder2MotorType;
        this.m_Encoder2Wheel = Encoder2Wheel;
    }

    public Robot2Wheel2DTracker(Robot2Wheel2DTracker oldTracker) {
        super(oldTracker);
        this.m_Encoder1Reversed = oldTracker.m_Encoder1Reversed;
        this.m_Encoder1MotorType = oldTracker.m_Encoder1MotorType;
        this.m_Encoder1Wheel = oldTracker.m_Encoder1Wheel;
        this.m_Encoder2Reversed = oldTracker.m_Encoder2Reversed;
        this.m_Encoder2MotorType = oldTracker.m_Encoder2MotorType;
        this.m_Encoder2Wheel = oldTracker.m_Encoder2Wheel;
    }


    protected abstract void updateData();
    protected abstract int getEncoder1Count();
    protected abstract int getEncoder2Count();
    protected abstract float getGyroReading();
    protected abstract boolean isGyroCounterClockwisePositive();

    @Override
    public void __trackStart() {
        c_ENCODER1_COUNTS_PER_CM = m_Encoder1MotorType.getCountsPerRev() / m_Encoder1Wheel.getCircumference();
        c_ENCODER2_COUNTS_PER_CM = m_Encoder2MotorType.getCountsPerRev() / m_Encoder2Wheel.getCircumference();

        c_ENCODER1_THETA_ANG = Math.toDegrees(Math.atan2(m_Encoder1Wheel.getOnRobotPosition().Y,m_Encoder1Wheel.getOnRobotPosition().X));
        c_ENCODER1_BETA_ANG = 90 + c_ENCODER1_THETA_ANG - m_Encoder1Wheel.getOnRobotPosition().getRotationZ();
        c_ENCODER2_THETA_ANG = Math.toDegrees(Math.atan2(m_Encoder2Wheel.getOnRobotPosition().Y,m_Encoder2Wheel.getOnRobotPosition().X));
        c_ENCODER2_BETA_ANG = 90 + c_ENCODER2_THETA_ANG - m_Encoder2Wheel.getOnRobotPosition().getRotationZ();
        c_ENCODER1_CM_PER_DEG = Math.cos(Math.toRadians(c_ENCODER1_BETA_ANG)) * (XYPlaneCalculations.CONST_PI_OVER_180 * m_Encoder1Wheel.getCircumference());
        c_ENCODER2_CM_PER_DEG = Math.cos(Math.toRadians(c_ENCODER2_BETA_ANG)) * (XYPlaneCalculations.CONST_PI_OVER_180 * m_Encoder2Wheel.getCircumference());

        double tmpAlpha0AngInRad = Math.toRadians(m_Encoder1Wheel.getOnRobotPosition().getRotationZ());
        c_TRANSFORMATION_MATRIX[0][0] = Math.cos(tmpAlpha0AngInRad);
        c_TRANSFORMATION_MATRIX[0][1] = Math.sin(tmpAlpha0AngInRad);
        double tmpAlpha1AngInRad = Math.toRadians(m_Encoder2Wheel.getOnRobotPosition().getRotationZ());
        c_TRANSFORMATION_MATRIX[1][0] = Math.cos(tmpAlpha1AngInRad);
        c_TRANSFORMATION_MATRIX[1][1] = Math.sin(tmpAlpha1AngInRad);
        if(c_TRANSFORMATION_MATRIX[0][0] != 0 && c_TRANSFORMATION_MATRIX[1][0] != 0){
            //Both tracking wheels provide X axis values, divide them by 2
            c_TRANSFORMATION_MATRIX[0][0] /= 2.0;
            c_TRANSFORMATION_MATRIX[1][0] /= 2.0;
        }
        if(c_TRANSFORMATION_MATRIX[0][1] != 0 && c_TRANSFORMATION_MATRIX[1][1] != 0){
            //Both tracking wheels provide Y axis values, divide them by 2
            c_TRANSFORMATION_MATRIX[0][1] /= 2.0;
            c_TRANSFORMATION_MATRIX[1][1] /= 2.0;
        }

        this.updateData();

        m_LastEncoder1Count = this.getEncoder1Count();
        m_LastEncoder2Count = this.getEncoder2Count();

        m_LastGyroReading = this.getGyroReading();
    }

    @Override
    public void __trackLoop(double secondsSinceLastLoop) {
        this.updateData();
        int newEncoder1Count = this.getEncoder1Count();
        int newEncoder2Count = this.getEncoder2Count();
        float newGyroReading = this.getGyroReading();

        int deltaEncoder1Count = newEncoder1Count - m_LastEncoder2Count;
        int deltaEncoder2Count = newEncoder2Count - m_LastEncoder1Count;

        if(m_Encoder2Reversed)
            deltaEncoder2Count = -deltaEncoder2Count;
        if(m_Encoder1Reversed)
            deltaEncoder1Count = -deltaEncoder1Count;

        double deltaEncoder1CM = deltaEncoder1Count / c_ENCODER1_COUNTS_PER_CM;
        double deltaEncoder2CM = deltaEncoder2Count / c_ENCODER2_COUNTS_PER_CM;
        double deltaAngMoved = newGyroReading - m_LastGyroReading;
        if(!this.isGyroCounterClockwisePositive()){
            deltaAngMoved = -deltaAngMoved;
        }
        //deltaAngMoved = getDeltaAng(deltaAngMoved);
        deltaAngMoved = XYPlaneCalculations.normalizeDeg(deltaAngMoved);

        double deltaEncoder1AxialCM = deltaEncoder1CM - deltaAngMoved * c_ENCODER1_CM_PER_DEG;
        double deltaEncoder2AxialCM = deltaEncoder2CM - deltaAngMoved * c_ENCODER2_CM_PER_DEG;
        double deltaXMoved = c_TRANSFORMATION_MATRIX[0][0] * deltaEncoder1AxialCM + c_TRANSFORMATION_MATRIX[1][0] * deltaEncoder2AxialCM;
        double deltaYMoved = c_TRANSFORMATION_MATRIX[0][1] * deltaEncoder1AxialCM + c_TRANSFORMATION_MATRIX[1][1] * deltaEncoder2AxialCM;

        deltaXMoved /= this.getPositionTracker().getXDistanceFactor();
        deltaYMoved /= this.getPositionTracker().getYDistanceFactor();

        RobotPose2D currentVelocityVector = new RobotPose2D(
                deltaXMoved / secondsSinceLastLoop,
                deltaYMoved / secondsSinceLastLoop,
                deltaAngMoved / secondsSinceLastLoop
        );

        this.getPositionTracker().__trackLoopMovedRaw(
                currentVelocityVector,
                new RobotPose2D(
                    deltaXMoved,
                    deltaYMoved,
                    deltaAngMoved
                )
        );

        m_LastEncoder2Count = newEncoder2Count;
        m_LastEncoder1Count = newEncoder1Count;
        m_LastGyroReading = newGyroReading;
    }

    public void __trackLoop_NoActualPositionShift(double secondsSinceLastLoop){
        this.updateData();
        int newEncoder1Count = this.getEncoder1Count();
        int newEncoder2Count = this.getEncoder2Count();
        float newGyroReading = this.getGyroReading();
        m_LastEncoder2Count = newEncoder2Count;
        m_LastEncoder1Count = newEncoder1Count;
        m_LastGyroReading = newGyroReading;
    }
}
