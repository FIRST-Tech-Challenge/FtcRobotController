package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.io.InputStream;

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
public class AssetsTrajectoryManager {

    /**
     * Loads the group config.
     */
    public static @Nullable
    TrajectoryGroupConfig loadGroupConfig() {
        try {
            InputStream inputStream = AppUtil.getDefContext().getAssets().open(
                    "trajectory/" + TrajectoryConfigManager.GROUP_FILENAME);
            return TrajectoryConfigManager.loadGroupConfig(inputStream);
        } catch (IOException e) {
            return null;
        }
    }

    /**
     * Loads a trajectory config with the given name.
     */
    public static @Nullable TrajectoryConfig loadConfig(String name) {
        try {
            InputStream inputStream = AppUtil.getDefContext().getAssets().open(
                    "trajectory/" + name + ".yaml");
            return TrajectoryConfigManager.loadConfig(inputStream);
        } catch (IOException e) {
            return null;
        }
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        TrajectoryGroupConfig groupConfig = loadGroupConfig();
        TrajectoryConfig config = loadConfig(name);
        if (groupConfig == null || config == null) {
            return null;
        }
        return new HALTrajectoryBuilder(config.toTrajectoryBuilder(groupConfig), coordinateMode, distanceUnit, angleUnit);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit) {
        return loadBuilder(name, coordinateMode, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, CoordinateMode coordinateMode, HALAngleUnit angleUnit) {
        return loadBuilder(name, coordinateMode, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, HALDistanceUnit distanceUnit) {
        return loadBuilder(name, CoordinateMode.HAL, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, HALAngleUnit angleUnit) {
        return loadBuilder(name, CoordinateMode.HAL, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name, CoordinateMode coordinateMode) {
        return loadBuilder(name, coordinateMode, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    public static @Nullable HALTrajectoryBuilder loadBuilder(String name) {
        return loadBuilder(name, CoordinateMode.HAL, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        HALTrajectoryBuilder builder = loadBuilder(name, coordinateMode);
        if (builder == null) {
            return null;
        }
        return builder.build();
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit) {
        return load(name, coordinateMode, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, CoordinateMode coordinateMode, HALAngleUnit angleUnit) {
        return load(name, coordinateMode, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, HALDistanceUnit distanceUnit) {
        return load(name, CoordinateMode.HAL, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, HALAngleUnit angleUnit) {
        return load(name, CoordinateMode.HAL, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name, CoordinateMode coordinateMode) {
        return load(name, coordinateMode, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable HALTrajectory load(String name) {
        return load(name, CoordinateMode.HAL, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

}
