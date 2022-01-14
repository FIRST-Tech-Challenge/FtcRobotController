package org.firstinspires.ftc.teamcode.lib.kinematics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

public class HolonomicOdometry extends Odometry {
    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private final double centerWheelOffset;

    // the suppliers
    DoubleSupplier m_left, m_right, m_horizontal;

    public HolonomicOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset, Pose2d initialPose) {
        super(initialPose, trackWidth);
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
        m_left = leftEncoder;
        m_right = rightEncoder;
        m_horizontal = horizontalEncoder;
    }

    public HolonomicOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {
        this(leftEncoder, rightEncoder, horizontalEncoder, trackWidth, centerWheelOffset, new Pose2d());
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble());
    }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = m_left.getAsDouble();
        prevRightEncoder = m_right.getAsDouble();
        prevHorizontalEncoder = m_horizontal.getAsDouble();
    }

    private void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        -(deltaLeftEncoder - deltaRightEncoder) / trackWidth
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = deltaHorizontalEncoder - (centerWheelOffset * dw);
        double dy = (deltaLeftEncoder + deltaRightEncoder) / 2;

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}
