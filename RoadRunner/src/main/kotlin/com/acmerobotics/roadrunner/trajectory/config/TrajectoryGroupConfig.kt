package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.fasterxml.jackson.annotation.JsonIgnore

/**
 * Configuration describing constraints and other robot-specific parameters that are shared by a group of trajectories.
 */
data class TrajectoryGroupConfig(
    val maxVel: Double,
    val maxAccel: Double,
    val maxAngVel: Double,
    val maxAngAccel: Double,
    val robotLength: Double,
    val robotWidth: Double,
    val driveType: DriveType,
    val trackWidth: Double?,
    val wheelBase: Double?,
    val lateralMultiplier: Double?
) {
    /**
     * Type of drivetrain.
     */
    enum class DriveType {
        GENERIC,
        MECANUM,
        TANK
    }

    @JsonIgnore val velConstraint: TrajectoryVelocityConstraint =
        MinVelocityConstraint(
            listOf(
                AngularVelocityConstraint(maxAngVel),
                when (driveType) {
DriveType.GENERIC -> TranslationalVelocityConstraint(maxVel)
DriveType.MECANUM -> MecanumVelocityConstraint(
    maxVel,
    trackWidth!!,
    wheelBase ?: trackWidth,
    lateralMultiplier!!
)
DriveType.TANK -> TankVelocityConstraint(maxVel, trackWidth!!, wheelBase ?: trackWidth)
}
            )
        )

    @JsonIgnore val accelConstraint: TrajectoryAccelerationConstraint =
        ProfileAccelerationConstraint(maxAccel)
}
