package org.firstinspires.ftc.teamcode.subsystem.turret

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystem.Subsystem

sealed class Turret : Subsystem() {
    companion object {
        const val ANGLE_TOLERANCE: Double = 1.0
        const val MIN_ANGLE: Double = -120.0
        const val MAX_ANGLE: Double = 120.0
    }
    open var angle: Double = 0.0
        set(angle) {
            field = AngleUnit.normalizeDegrees(angle)
        }
    abstract val isWithinTolerance: Boolean
    fun off() {
        angle = 0.0
    }
    init {
        off()
    }
}