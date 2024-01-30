package org.firstinspires.ftc.teamcode.utils;
import org.firstinspires.ftc.teamcode.utils.geometry.*;

import java.util.function.DoubleSupplier;

public class HolonomicOdometry   {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private BTRotation2d previousAngle;
    private double centerWheelOffset;
    protected BTPose2d robotPose = new BTPose2d();

    public BTPose2d getPose() {
        return robotPose;
    }


    /**
     * The trackwidth of the odometers
     */
    protected double m_trackWidth;

    // the suppliers
    DoubleSupplier m_left, m_right, m_horizontal, m_gyro;

    public HolonomicOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, DoubleSupplier gyroAngle, double trackWidth, double centerWheelOffset) {
        this(trackWidth, centerWheelOffset);
        m_left = leftEncoder;
        m_right = rightEncoder;
        m_horizontal = horizontalEncoder;
        m_gyro = gyroAngle;
    }

    public HolonomicOdometry(BTPose2d initialPose, double trackwidth, double centerWheelOffset) {
        this.m_trackWidth = trackwidth;
        previousAngle = initialPose.getRotation();
        robotPose=initialPose;
        this.centerWheelOffset = centerWheelOffset;
    }

    public HolonomicOdometry(double trackwidth, double centerWheelOffset) {
        this(new BTPose2d(), trackwidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble(), m_gyro.getAsDouble());
    }

    public void setPose(BTPose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos,double gyroAngle) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        BTRotation2d angle = BTRotation2d.fromDegrees(gyroAngle);

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        BTTwist2d twist2d = new BTTwist2d(dx, dy, dw);

        BTPose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new BTPose2d(newPose.getTranslation(), angle);
    }

    public void reset(BTPose2d btPose2d) {
        previousAngle=BTRotation2d.fromDegrees(0);
        prevLeftEncoder=0;
        prevRightEncoder=0;
        prevHorizontalEncoder=0;
        robotPose=btPose2d;
    }
}
