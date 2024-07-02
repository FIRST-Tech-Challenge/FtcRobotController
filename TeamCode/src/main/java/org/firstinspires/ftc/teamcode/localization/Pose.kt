package org.firstinspires.ftc.teamcode.localization

data class Pose(
    @get:JvmName("x") val x: Double,
    @get:JvmName("y") val y: Double,
    @get:JvmName("heading") val heading: Double,
) {
    /**
     * vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     */
    @JvmName("add")
    operator fun plus(other: Pose) = Pose(x + other.x, y + other.y, heading + other.heading)

    @JvmOverloads
    constructor(x: Number, y: Number, heading: Number = 0.0) : this(
        x.toDouble(),
        y.toDouble(),
        heading.toDouble()
    )
}

data class Motion(
    val forward: Double,
    val right: Double,
    val turn: Double
) {

}