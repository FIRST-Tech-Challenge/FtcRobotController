package teamcode.v1.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 496.0/90.0
    @JvmField var homePos = -68.0
    @JvmField var autoHomePos = 223.8
    @JvmField var intervalPos = -50.0
    @JvmField var groundPos = -68.0
    @JvmField var lowPos = 165.0
    @JvmField var midPos = 150.0
    @JvmField var highPos = 150.0
    @JvmField var kP = 0.015
    @JvmField var kI = 0.00
    @JvmField var kD = 0.0015
    @JvmField var kS = 0.0
    @JvmField var kV = 0.00001
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.1
    @JvmField var maxVel = 500.0
    @JvmField var maxAccel = 1000.0
    @JvmField var maxDeccel = 1000.0
    @JvmField var allowedPositionError = 0.0
    @JvmField var disabledPosition = -68.0
}