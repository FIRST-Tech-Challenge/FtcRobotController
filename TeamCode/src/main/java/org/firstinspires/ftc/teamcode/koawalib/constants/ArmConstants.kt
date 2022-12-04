package org.firstinspires.ftc.teamcode.koawalib.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 295.0/90.0
    @JvmField var homePos = -67.7
    @JvmField var autoHomePos = 210.0
    @JvmField var groundPos = -60.0
    @JvmField var lowPos = -30.0
    @JvmField var midPos = 130.0
    @JvmField var highPos = 130.0
    @JvmField var kP = 0.01
    @JvmField var kI = 0.0
    @JvmField var kD = 0.00265
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.425
    @JvmField var maxVel = 0.0
    @JvmField var maxAccel = 0.0
    @JvmField var allowedPositionError = 1.0
}