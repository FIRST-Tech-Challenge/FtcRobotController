package org.firstinspires.ftc.teamcode.roadrunner.util;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
public class AssetsTrajectoryManager {

    /**
     * Loads the group config.
     */
//    public static @Nullable
//    TrajectoryGroupConfig loadGroupConfig() {
//        try {
//            File initialFile = new File("/sdcard/auton/" + TrajectoryConfigManager.GROUP_FILENAME);
//            InputStream inputStream = new FileInputStream(initialFile);
////            InputStream inputStream = AppUtil.getDefContext().getAssets().open(
////                    "trajectory/" + TrajectoryConfigManager.GROUP_FILENAME);
//            return TrajectoryConfigManager.loadGroupConfig(inputStream);
//        } catch (IOException e) {
//            return null;
//        }
//    }

    //PREVIOUS VERSION (prior to autonscriptparser)
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
//    public static @Nullable TrajectoryConfig loadConfig(String name) {
//        try {
//            File initialFile = new File("/sdcard/auton/" + name + ".yaml");
//            InputStream inputStream = new FileInputStream(initialFile);
////            InputStream inputStream = AppUtil.getDefContext().getAssets().open(
////                    "trajectory/" + name + ".yaml");
//            return TrajectoryConfigManager.loadConfig(inputStream);
//        } catch (IOException e) {
//            return null;
//        }
//    }
    //PREVIOUS version (prior to autonscriptparser)
    public static @Nullable TrajectoryBuilder loadBuilder(String name, double maxVel, double maxAccel) {
        TrajectoryGroupConfig groupConfig = loadGroupConfig();
        TrajectoryGroupConfig modifiedGroupConfig = groupConfig.copy(maxVel,
                maxAccel,
                groupConfig.getMaxAngVel(),
                groupConfig.getMaxAngAccel(),
                groupConfig.getRobotLength(),
                groupConfig.getRobotWidth(),
                groupConfig.getDriveType(),
                groupConfig.getTrackWidth(),
                groupConfig.getWheelBase(),
                groupConfig.getLateralMultiplier()
        );
        TrajectoryConfig config = loadConfig(name);
        if (groupConfig == null || config == null) {
            return null;
        }
        return config.toTrajectoryBuilder(modifiedGroupConfig);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable Trajectory load(String name, double maxVel, double maxAccel) {
        TrajectoryBuilder builder = loadBuilder(name, maxVel, maxAccel);
        if (builder == null) {
            return null;
        }
        return builder.build();
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
    public static @Nullable TrajectoryBuilder loadBuilder(String name) {
        TrajectoryGroupConfig groupConfig = loadGroupConfig();
        TrajectoryConfig config = loadConfig(name);
        if (groupConfig == null || config == null) {
            return null;
        }
        return config.toTrajectoryBuilder(groupConfig);
    }

    /**
     * Loads a trajectory with the given name.
     */
    public static @Nullable Trajectory load(String name) {
        TrajectoryBuilder builder = loadBuilder(name);
        if (builder == null) {
            return null;
        }
        return builder.build();
    }
}
