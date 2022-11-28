package org.firstinspires.ftc.teamcode.koawalib.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 186.0/90.0
    @JvmField var homePos = -78.8
    @JvmField var groundPos = -50.0
    @JvmField var lowPos = 198.0
    @JvmField var midPos = 135.0
    @JvmField var highPos = 135.0
    @JvmField var kP = 0.01
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0017
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.35
    @JvmField var maxVel = 0.0
    @JvmField var maxAccel = 0.0
    @JvmField var allowedPositionError = 1.0
}