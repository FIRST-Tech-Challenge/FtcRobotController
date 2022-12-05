package org.firstinspires.ftc.teamcode.koawalib.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 295.0/90.0
    @JvmField var homePos = -67.7
    @JvmField var autoHomePos = 210.0
    @JvmField var groundPos = -65.0
    @JvmField var lowPos = 180.0
    @JvmField var midPos = 125.0
    @JvmField var highPos = 125.0
    @JvmField var kP = 0.016
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0024
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.4
    @JvmField var maxVel = 0.0
    @JvmField var maxAccel = 0.0
    @JvmField var allowedPositionError = 2.0
}