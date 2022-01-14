package org.firstinspires.ftc.teamcode.lib.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.commandftc.RobotUniversal;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class RoadRunnerOdometry extends ThreeTrackingWheelLocalizer {
    private final DoubleSupplier[] encoders;
    private final DoubleSupplier[] velocity;
    public RoadRunnerOdometry(DoubleSupplier[] encoders) {
        super(Arrays.asList(
                new Pose2d(-0.105, 0.0954, 0), // left parallel
                new Pose2d(-0.105, -0.0954, 0), // right parallel
                new Pose2d(0.025, 0, Math.toRadians(-90)) // perpendicular
        ));
        this.encoders = encoders;
        velocity = null;
    }

    public RoadRunnerOdometry(DoubleSupplier[] encoders, DoubleSupplier[] velocity) {
        super(Arrays.asList(
                new Pose2d(-0.105, 0.0954, 0), // left parallel
                new Pose2d(-0.105, -0.0954, 0), // right parallel
                new Pose2d(0.025, 0, Math.toRadians(-90)) // perpendicular
        ));
        this.encoders = encoders;
        this.velocity  = velocity;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(encoders[0].getAsDouble(), encoders[1].getAsDouble(), encoders[2].getAsDouble());
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        if (velocity == null)
            return null;
        return Arrays.asList(velocity[0].getAsDouble(), velocity[1].getAsDouble(), velocity[2].getAsDouble());
    }
}
