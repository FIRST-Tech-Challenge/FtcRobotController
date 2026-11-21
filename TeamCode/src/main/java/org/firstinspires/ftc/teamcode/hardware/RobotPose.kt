package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

data class RobotPose(
    /**
     * inches
     */
    @JvmField var x: Double,
    /**
     * inches
     */
    @JvmField var y: Double,
    /**
     * radians
     */
    @JvmField var a: Double,
    /**
     * degrees
     */
    @JvmField val turret: Double? = null
) {
    @get:JvmName("asPose2D")
    val asPose2D: Pose2D get() = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, a)
}