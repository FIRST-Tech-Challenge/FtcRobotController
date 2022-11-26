package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.fasterxml.jackson.annotation.JsonInclude
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import java.io.File
import java.io.InputStream

/**
 * Class containing methods for saving (loading) trajectory configurations to (from) YAML files.
 */
object TrajectoryConfigManager {
    @JvmField
    @Suppress("MayBeConst")
    val GROUP_FILENAME = "_group.yaml"

    private val MAPPER = ObjectMapper(YAMLFactory())

    init {
        MAPPER.registerKotlinModule()
        MAPPER.setSerializationInclusion(JsonInclude.Include.NON_NULL)
    }

    /**
     * Saves a [TrajectoryConfig] to [file].
     */
    @JvmStatic
    fun saveConfig(trajectoryConfig: TrajectoryConfig, file: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    /**
     * Saves a [TrajectoryGroupConfig] to [dir].
     */
    @JvmStatic
    fun saveGroupConfig(trajectoryConfig: TrajectoryGroupConfig, dir: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(File(dir, GROUP_FILENAME), trajectoryConfig)
    }

    /**
     * Loads a [TrajectoryConfig] from [file].
     */
    @JvmStatic
    fun loadConfig(file: File): TrajectoryConfig? = MAPPER.readValue(file, TrajectoryConfig::class.java)

    /**
     * Loads a [TrajectoryConfig] from [inputStream].
     */
    @JvmStatic
    fun loadConfig(inputStream: InputStream): TrajectoryConfig? =
        MAPPER.readValue(inputStream, TrajectoryConfig::class.java)

    /**
     * Loads the [TrajectoryGroupConfig] inside [dir].
     */
    @JvmStatic
    fun loadGroupConfig(dir: File): TrajectoryGroupConfig? {
        val groupFile = File(dir, GROUP_FILENAME)
        if (!groupFile.exists()) {
            return null
        }
        return MAPPER.readValue(groupFile, TrajectoryGroupConfig::class.java)
    }

    /**
     * Loads the [TrajectoryGroupConfig] from [inputStream].
     */
    @JvmStatic
    fun loadGroupConfig(inputStream: InputStream) =
        MAPPER.readValue(inputStream, TrajectoryGroupConfig::class.java)

    /**
     * Loads a [TrajectoryBuilder] from [file].
     */
    @JvmStatic
    fun loadBuilder(file: File): TrajectoryBuilder? {
        val config = loadConfig(file) ?: return null
        return config.toTrajectoryBuilder(loadGroupConfig(file) ?: return null)
    }

    /**
     * Loads a [Trajectory] from [file].
     */
    @JvmStatic
    fun load(file: File) = loadBuilder(file)?.build()
}
