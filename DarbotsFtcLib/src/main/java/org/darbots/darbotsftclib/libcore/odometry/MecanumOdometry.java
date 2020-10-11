package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.OdometryMethod;

public class MecanumOdometry extends OdometryMethod {
    private int m_LastLTEncoderCount, m_LastRTEncoderCount, m_LastLBEncoderCount, m_LastRBEncoderCount;
    private double c_LT_COUNTS_PER_DEG, c_RT_COUNTS_PER_DEG, c_LB_COUNTS_PER_DEG, c_RB_COUNTS_PER_DEG;
    private RobotMotor m_LTMotor, m_RTMotor, m_LBMotor, m_RBMotor;

    private MecanumDrivetrain m_MotionSystem;

    public MecanumOdometry(MecanumDrivetrain driveTrain) {
        super();
        this.m_MotionSystem = driveTrain;
    }

    public MecanumOdometry(MecanumOdometry oldTracker) {
        super(oldTracker);
        this.m_MotionSystem = oldTracker.m_MotionSystem;
    }

    @Override
    public void __trackStart() {
        m_LTMotor = m_MotionSystem.getLTMotion().getMotor();
        m_RTMotor = m_MotionSystem.getRTMotion().getMotor();
        m_LBMotor = m_MotionSystem.getLBMotion().getMotor();
        m_RBMotor = m_MotionSystem.getRBMotion().getMotor();

        c_LT_COUNTS_PER_DEG = m_LTMotor.getMotorType().getCountsPerRev() / 360.0;
        c_RT_COUNTS_PER_DEG = m_RTMotor.getMotorType().getCountsPerRev() / 360.0;
        c_LB_COUNTS_PER_DEG = m_LBMotor.getMotorType().getCountsPerRev() / 360.0;
        c_RB_COUNTS_PER_DEG = m_RBMotor.getMotorType().getCountsPerRev() / 360.0;

        m_LastLTEncoderCount = m_LTMotor.getCurrentCount();
        m_LastRTEncoderCount = m_RTMotor.getCurrentCount();
        m_LastLBEncoderCount = m_LBMotor.getCurrentCount();
        m_LastRBEncoderCount = m_RBMotor.getCurrentCount();
    }

    @Override
    public void __trackLoop(double secondsSinceLastLoop) {
        int newLTCount = m_LTMotor.getCurrentCount();
        int newRTCount = m_RTMotor.getCurrentCount();
        int newLBCount = m_LBMotor.getCurrentCount();
        int newRBCount = m_RBMotor.getCurrentCount();

        int deltaLTCount = newLTCount - m_LastLTEncoderCount;
        int deltaRTCount = newRTCount - m_LastRTEncoderCount;
        int deltaLBCount = newLBCount - m_LastLBEncoderCount;
        int deltaRBCount = newRBCount - m_LastRBEncoderCount;

        double deltaLTDeg = deltaLTCount / c_LT_COUNTS_PER_DEG;
        double deltaRTDeg = deltaRTCount / c_RT_COUNTS_PER_DEG;
        double deltaLBDeg = deltaLBCount / c_LB_COUNTS_PER_DEG;
        double deltaRBDeg = deltaRBCount / c_RB_COUNTS_PER_DEG;

        double LTAngularSpeed = deltaLTDeg / secondsSinceLastLoop;
        double RTAngularSpeed = deltaRTDeg / secondsSinceLastLoop;
        double LBAngularSpeed = deltaLBDeg / secondsSinceLastLoop;
        double RBAngularSpeed = deltaRBDeg / secondsSinceLastLoop;

        double[] wheelSpeeds = {LTAngularSpeed, RTAngularSpeed, LBAngularSpeed, RBAngularSpeed};
        RobotVector2D chassisSpeed = m_MotionSystem.calculateRawRobotSpeed(wheelSpeeds);


        double deltaXMoved = chassisSpeed.X * secondsSinceLastLoop;
        double deltaYMoved = chassisSpeed.Y * secondsSinceLastLoop;
        double deltaAngMoved = chassisSpeed.getRotationZ() * secondsSinceLastLoop;

        deltaAngMoved = this.getPositionTracker().__getDeltaAng(deltaAngMoved);

        this.getPositionTracker().__trackLoopMoved(
                chassisSpeed,
                new RobotPose2D(
                    deltaXMoved,
                    deltaYMoved,
                    deltaAngMoved
                )
        );

        m_LastLTEncoderCount = newLTCount;
        m_LastRTEncoderCount = newRTCount;
        m_LastLBEncoderCount = newLBCount;
        m_LastRBEncoderCount = newRBCount;
    }

    @Override
    public void __trackLoop_NoActualPositionShift(double secondsSinceLastLoop) {
        int newLTCount = m_LTMotor.getCurrentCount();
        int newRTCount = m_RTMotor.getCurrentCount();
        int newLBCount = m_LBMotor.getCurrentCount();
        int newRBCount = m_RBMotor.getCurrentCount();
        m_LastLTEncoderCount = newLTCount;
        m_LastRTEncoderCount = newRTCount;
        m_LastLBEncoderCount = newLBCount;
        m_LastRBEncoderCount = newRBCount;
    }

}
