package org.firstinspires.ftc.teamcode.util.math

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.util.Waypoint

data class Pose(
    var p: Point,
    var h: Angle
) {
    val x get() = p.x
    val y get() = p.y
    val cos get() = h.cos
    val sin get() = h.sin
    val hypot get() = p.hypot
    val copy get() = Pose(p, h)

    val pose2d get() = Pose2d(p.x, p.y, h.angle)

    fun distance(p2: Pose) = p.distance(p2.p)
    fun distance(p2: Waypoint) = p.distance(p2.p)

    val toRawString = String.format("%.2f, %.2f, %.2f", x, y, h.angle)
    override fun toString() = String.format("%.2f, %.2f, %.2f", x, y, h.wrap().deg)

    companion object {
        val DEFAULT_ANGLE = Pose(Point.ORIGIN, Angle.EAST)
        val DEFAULT_RAW = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAW))
    }
}
