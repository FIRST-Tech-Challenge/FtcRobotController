package org.firstinspires.ftc.teamcode.koawalib.constants

import com.acmerobotics.dashboard.config.Config

@Config
object LiftConstants {
    @JvmField var ticksPerUnit = 2020.0/17.25
    @JvmField var homePos = 0.0
    @JvmField var groundPos = 0.0
    @JvmField var lowPos = 0.0
    @JvmField var midPos = 4.5
    @JvmField var highPos = 14.5
    @JvmField var kP = 0.5
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kG = 0.1
    @JvmField var maxVel = 80.0
    @JvmField var maxAccel = 80.0
    @JvmField var disabledPosition = 0.0
    @JvmField var allowedPositionError = 1.0
}