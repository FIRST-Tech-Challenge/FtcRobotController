package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

import java.util.function.DoubleUnaryOperator;

public class CurvePoint {
    private Pose2d pose;
    private DoubleUnaryOperator lookaheadDistance;

    public CurvePoint(Pose2d pose, DoubleUnaryOperator lookaheadDistance) {
        setPose(pose);
        setLookaheadDistance(lookaheadDistance);
    }

    public CurvePoint(Pose2d pose, float lookaheadDistance) {
        this(pose, (speed) -> lookaheadDistance);
    }

    public CurvePoint(Pose2d pose, double kP_Lookahead) {
        this(pose, (speed) -> kP_Lookahead * speed);
    }

    public CurvePoint(double x, double y, double radians, DoubleUnaryOperator lookaheadDistance) {
        this(new Pose2d(x, y, new Rotation2d(radians, true)), lookaheadDistance);
    }

    public CurvePoint(double x, double y, double radians, float lookaheadDistance) {
        this(x, y, radians, (speed) -> lookaheadDistance);
    }

    public CurvePoint(double x, double y, double radians, double kP_Lookahead) {
        this(x, y, radians, (speed) -> kP_Lookahead * speed);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public double getLookaheadDistance(double robotSpeed) {
        return lookaheadDistance.applyAsDouble(robotSpeed);
    }

    public void setLookaheadDistance(DoubleUnaryOperator lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }
}
