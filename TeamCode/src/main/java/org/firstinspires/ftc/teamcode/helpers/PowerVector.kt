package org.firstinspires.ftc.teamcode.helpers

data class PowerVector(
    val axial: Double,
    val lateral: Double,
    val yaw: Double,
) {
    init {
        require(axial <= 1f) { "Axial Power cannot exceed 1.0" }
        require(axial >= -1f) { "Axial Power cannot be less than -1.0" }
        require(lateral <= 1f) { "Lateral Power cannot exceed 1.0" }
        require(lateral >= -1f) { "Lateral Power cannot be less than -1.0" }
        require(yaw <= 1f) { "Yaw Power cannot exceed 1.0" }
        require(yaw >= -1f) { "Yaw Power cannot be less than -1.0" }
    }

    fun leftFrontPower() = (axial + lateral + yaw)
    fun rightFrontPower() = (axial - lateral - yaw)
    fun leftBackPower() = (axial - lateral + yaw)
    fun rightBackPower() = (axial + lateral - yaw)
}
