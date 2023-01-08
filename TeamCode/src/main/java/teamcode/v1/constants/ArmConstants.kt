package teamcode.v1.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 353.0/90.0
    @JvmField var homePos = -71.0
    @JvmField var autoHomePos = 227.0
    @JvmField var intervalPos = -10.0
    @JvmField var groundPos = -70.0
    @JvmField var lowPos = 170.0
    @JvmField var midPos = 135.0
    @JvmField var highPos = 150.0
    @JvmField var kP = 0.01
    @JvmField var kI = 0.00
    @JvmField var kD = 0.00025
    @JvmField var kS = 0.0
    @JvmField var kV = 0.0
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.25
    @JvmField var maxVel = 0.0
    @JvmField var maxAccel = 0.0
    @JvmField var allowedPositionError = 5.0
    @JvmField var disabledPosition = -71.0
}