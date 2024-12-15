package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.rowlandhall.meepmeep.MeepMeep
import org.rowlandhall.meepmeep.core.colorscheme.ColorScheme
import org.rowlandhall.meepmeep.roadrunner.AddTrajectorySequenceCallback
import org.rowlandhall.meepmeep.roadrunner.Constraints
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence

/**
 * [ActualBotBuilder] is a builder class for creating instances of
 * [RoadRunnerBotEntity]. It allows setting various properties such as
 * dimensions, start pose, constraints, drive train type, and color scheme.
 *
 * @property meepMeep The [MeepMeep] instance used for building the bot
 *    entity.
 */
open class ActualBotBuilder(private val meepMeep: MeepMeep) {
    /** The constraints for the bot's trajectory. */
    private var constraints = Constraints(
        30.0, 30.0, Math.toRadians(60.0), Math.toRadians(60.0), 15.0
    )

    /** The width of the bot. */
    private var width = 15.0

    /** The height of the bot. */
    private var height = 18.0

    /** The starting pose of the bot. */
    private var startPose = Pose2d()

    /** The color scheme of the bot. */
    private var colorScheme: ColorScheme? = null

    /** The opacity of the bot. */
    private var opacity = 0.8

    /** The drive train type of the bot. */
    private var driveTrainType = DriveTrainType.MECANUM

    /**
     * Sets the dimensions of the bot.
     *
     * @param width The width of the bot.
     * @param height The height of the bot.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setDimensions(width: Double, height: Double): ActualBotBuilder {
        this.width = width
        this.height = height

        return this
    }

    /**
     * Sets the starting pose of the bot.
     *
     * @param pose The [Pose2d] object representing the starting pose.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setStartPose(pose: Pose2d): ActualBotBuilder {
        this.startPose = pose

        return this
    }

    /**
     * Sets the constraints for the bot's trajectory.
     *
     * @param constraints The [Constraints] object containing the trajectory
     *    constraints.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setConstraints(constraints: Constraints): ActualBotBuilder {
        this.constraints = constraints

        return this
    }

    /**
     * Sets the constraints for the bot's trajectory.
     *
     * @param maxVel The maximum velocity.
     * @param maxAccel The maximum acceleration.
     * @param maxAngVel The maximum angular velocity.
     * @param maxAngAccel The maximum angular acceleration.
     * @param trackWidth The track width.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setConstraints(
        maxVel: Double, maxAccel: Double, maxAngVel: Double, maxAngAccel: Double, trackWidth: Double
    ): ActualBotBuilder {
        constraints = Constraints(maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth)

        return this
    }


    /**
     * Sets the drive train type of the bot.
     *
     * @param driveTrainType The [DriveTrainType] to set for the bot.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setDriveTrainType(driveTrainType: DriveTrainType): ActualBotBuilder {
        this.driveTrainType = driveTrainType

        return this
    }

    /**
     * Sets the color scheme of the bot.
     *
     * @param scheme The [ColorScheme] to set for the bot.
     * @return The current instance of [ActualBotBuilder] for chaining.
     */
    fun setColorScheme(scheme: ColorScheme): ActualBotBuilder {
        this.colorScheme = scheme

        return this
    }

    /**
     * Builds a new instance of [RoadRunnerBotEntity] using the current
     * configuration of the [ActualBotBuilder].
     *
     * @return A new [RoadRunnerBotEntity] instance.
     */
    private fun build(): RoadRunnerBotEntity {
        return RoadRunnerBotEntity(
            meepMeep,
            constraints,
            width,
            height,
            startPose,
            colorScheme ?: meepMeep.colorManager.theme,
            opacity,
            driveTrainType,
            false
        )
    }

    /**
     * Follows the given [TrajectorySequence] with the bot.
     *
     * @param trajectorySequence The [TrajectorySequence] to follow.
     * @return A [RoadRunnerBotEntity] instance that follows the given
     *    trajectory sequence.
     */
    fun followTrajectorySequence(trajectorySequence: TrajectorySequence): RoadRunnerBotEntity {
        val bot = this.build()
        bot.followTrajectorySequence(trajectorySequence)

        return bot
    }

    /**
     * Follows the trajectory sequence built by the provided callback.
     *
     * @param callback The [AddTrajectorySequenceCallback] used to build the
     *    trajectory sequence.
     * @return A [RoadRunnerBotEntity] instance that follows the built
     *    trajectory sequence.
     */
    fun followTrajectorySequence(callback: AddTrajectorySequenceCallback): RoadRunnerBotEntity {
        return followTrajectorySequence(callback.buildTrajectorySequence(build().drive))
    }
}
