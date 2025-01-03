package org.firstinspires.ftc.teamcode.utils;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.geometry.*;

import java.util.function.DoubleSupplier;

public class HolonomicOdometry {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private double centerWheelOffset;
    protected BTPose2d robotPose = new BTPose2d();
    private double dx;  // not null
    private double dt;// not null
    private double dv;
    private ElapsedTime time;
    private double previousTime;
    private double prevVelocity = 0;
    boolean isFirstTime = true;

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
        prevLeftEncoder = leftEncoder.getAsDouble();
        prevRightEncoder = rightEncoder.getAsDouble();
        prevHorizontalEncoder = horizontalEncoder.getAsDouble();

    }

    public HolonomicOdometry(BTPose2d initialPose, double trackwidth, double centerWheelOffset) {
        this.m_trackWidth = trackwidth;
        previousAngle = initialPose.getRotation();
        robotPose = initialPose;
        this.centerWheelOffset = centerWheelOffset;
        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousTime = time.time();
    }

    public HolonomicOdometry(double trackwidth, double centerWheelOffset) {
        this(new BTPose2d(0, 0, BTRotation2d.fromDegrees(0)), trackwidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble());
    }

    public void setPose(BTPose2d pose) {
//        previousAngle = pose.getRotation();
        robotPose = pose;


    }

    public void updateAngleByGyro() {
        previousAngle = Rotation2d.fromDegrees(m_gyro.getAsDouble());
        robotPose = new BTPose2d(robotPose.getX(), robotPose.getY(), BTRotation2d.fromDegrees(m_gyro.getAsDouble()));
    }

    boolean flag = true;

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;
        if (flag) {
            FtcDashboard.getInstance().getTelemetry().addData("delta  right", deltaRightEncoder);

            FtcDashboard.getInstance().getTelemetry().addData("delta  left", deltaLeftEncoder);
            FtcDashboard.getInstance().getTelemetry().addData("delta  center", deltaHorizontalEncoder);
            FtcDashboard.getInstance().getTelemetry().addData("delta  left-right", (deltaLeftEncoder - deltaRightEncoder) / m_trackWidth);
            flag = false;
            deltaHorizontalEncoder=0;
            deltaRightEncoder=0;
            deltaLeftEncoder=0;
        }
        Rotation2d angle = Rotation2d.fromDegrees(m_gyro.getAsDouble());
        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        dt = time.time() - previousTime;
        double dw = (angle.minus(previousAngle).getRadians());
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);
        dx = (deltaLeftEncoder + deltaRightEncoder) / 2;

        dv = getVelocity() - prevVelocity;
        prevVelocity = getVelocity();
        BTTwist2d twist2d = new BTTwist2d(dx, dy, dw);

        BTPose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;
        previousTime = time.time();
        robotPose = new BTPose2d(newPose.getTranslation(), BTRotation2d.fromDegrees(angle.getDegrees()));
        isFirstTime = false;
    }

    public void reset(BTPose2d btPose2d) {
        previousAngle = btPose2d.getRotation();
        robotPose = btPose2d;
    }

    public double getVelocity() {
        double velocity;
        if (!isFirstTime) {
            velocity = 0;
        } else {
            velocity = dx / dt;
        }
        return velocity;
    }

    public double getAcceleration() {
        double acceleration;
        if (!isFirstTime) {
            acceleration = 0;
        } else {
            acceleration = dv / dt;
        }
        return acceleration;
    }


}