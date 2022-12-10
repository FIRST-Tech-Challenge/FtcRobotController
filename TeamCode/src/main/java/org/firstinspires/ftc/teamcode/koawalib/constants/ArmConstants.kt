package org.firstinspires.ftc.teamcode.koawalib.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 700.0/90.0
    @JvmField var homePos = -70.0
    @JvmField var autoHomePos = 219.0
    @JvmField var groundPos = -65.0
    @JvmField var lowPos = 170.0
    @JvmField var midPos = 155.0
    @JvmField var highPos = 155.0
    @JvmField var kP = 0.025
    @JvmField var kI = 0.0
    @JvmField var kD = 0.005
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.1
    @JvmField var maxVel = 0.0
    @JvmField var maxAccel = 0.0
    @JvmField var allowedPositionError = 2.0
}