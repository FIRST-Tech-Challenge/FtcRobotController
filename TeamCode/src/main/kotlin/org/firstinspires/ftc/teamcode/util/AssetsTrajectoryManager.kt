package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.IOException

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
object AssetsTrajectoryManager {
    /**
     * Loads the group config.
     */
    fun loadGroupConfig(): TrajectoryGroupConfig? {
        try {
            val inputStream = AppUtil.getDefContext().assets.open(
                "trajectory/" + TrajectoryConfigManager.GROUP_FILENAME
            )
            return TrajectoryConfigManager.loadGroupConfig(inputStream)
        } catch (e: IOException) {
            return null
        }
    }

    /**
     * Loads a trajectory config with the given name.
     */
    fun loadConfig(name: String): TrajectoryConfig? {
        try {
            val inputStream = AppUtil.getDefContext().assets.open(
                "trajectory/$name.yaml"
            )
            return TrajectoryConfigManager.loadConfig(inputStream)
        } catch (e: IOException) {
            return null
        }
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    fun loadBuilder(name: String): TrajectoryBuilder? {
        val groupConfig: TrajectoryGroupConfig? = loadGroupConfig()
        val config: TrajectoryConfig? = loadConfig(name)
        if (groupConfig == null || config == null) {
            return null
        }
        return config.toTrajectoryBuilder(groupConfig)
    }

    /**
     * Loads a trajectory with the given name.
     */
    fun load(name: String): Trajectory? {
        val builder: TrajectoryBuilder = loadBuilder(name)
            ?: return null
        return builder.build()
    }
}