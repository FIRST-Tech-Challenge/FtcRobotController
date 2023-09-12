package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.function.DoubleFunction;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Function;

public class ConfigurablePose extends ConfigurableVector {

    public double heading;

    public ConfigurablePose(Pose2d pose) {
        super(pose.vec());
        heading = pose.getHeading();
    }

    public ConfigurablePose(double x, double y, double heading) {
        this(new Pose2d(x, y, heading));
    }

    public ConfigurablePose(double x, double y) {
        this(new Pose2d(x, y));
    }

    public ConfigurablePose(Vector2d vec) {
        this(vec.getX(), vec.getY());
    }

    public ConfigurablePose(Vector2d vec, double heading) {
        this(vec.getX(), vec.getY(), heading);
    }

    public ConfigurablePose() {
        this(new Pose2d());
    }

    @Override
    public ConfigurablePose set(double newX, double newY) {
        return (ConfigurablePose) super.set(newX, newY);
    }

    @Override
    public ConfigurablePose set(Vector2d vec) {
        return (ConfigurablePose) super.set(vec);
    }

    @Override
    public ConfigurablePose setX(double val) {
        return (ConfigurablePose) super.setX(val);
    }

    @Override
    public ConfigurablePose setY(double val) {
        return (ConfigurablePose) super.setY(val);
    }

    @Override
    public ConfigurablePose mutateVec(Function<Vector2d, Vector2d> callback) {
        return (ConfigurablePose) super.mutateVec(callback);
    }

    @Override
    public ConfigurablePose mutateX(DoubleFunction<Double> callback) {
        return (ConfigurablePose) super.mutateX(callback);
    }

    @Override
    public ConfigurablePose mutateY(DoubleFunction<Double> callback) {
        return (ConfigurablePose) super.mutateY(callback);
    }

    public ConfigurablePose set(double newX, double newY, double newHeading) {
        return set(new Pose2d(newX, newY, newHeading));
    }

    public ConfigurablePose set(Pose2d pose) {
        set(pose.vec());
        heading = pose.getHeading();
        return this;
    }

    public ConfigurablePose set(Vector2d vec, double newHeading) {
        set(vec);
        heading = newHeading;
        return this;
    }

    public ConfigurablePose setHeading(double val) {
        heading = val;
        return this;
    }

    public ConfigurablePose mutatePose(Function<Pose2d, Pose2d> callback) {
        return set(mapPose(callback));
    }

    public ConfigurablePose mutateHeading(DoubleFunction<Double> callback) {
        return setHeading(mapHeading(callback));
    }

    public <T> T mapPose(Function<Pose2d, T> callback) {
        return callback.apply(toPose());
    }

    private <T> T mapHeading(DoubleFunction<T> callback) {
        return callback.apply(getHeading());
    }

    public Pose2d toPose() {
        return new Pose2d(x, y, heading);
    }

    public double getHeading() {
        return heading;
    }

    public static Pose2d mirrorOverX(Pose2d old) {
        return new Pose2d(old.getX(), old.times(-1).getY(), old.times(-1).getHeading());
    }

    public static Pose2d mirrorOverY(Pose2d old) {
        return new Pose2d(
            old.times(-1).getX(),
            old.getY(),
            old.headingVec().rotated(-Math.PI / 2).times(-1).rotated(Math.PI / 2).angle()
        );
    }
}
