package teamcode.v1.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var ticksPerUnit = 496.0/90.0
    @JvmField var homePos = -71.0
    @JvmField var autoHomePos = 227.0
    @JvmField var intervalPos = -50.0
    @JvmField var groundPos = -70.0
    @JvmField var lowPos = 200.0
    @JvmField var midPos = 135.0
    @JvmField var highPos = 150.0
    @JvmField var kP = 0.02
    @JvmField var kI = 0.00
    @JvmField var kD = 0.002
    @JvmField var kS = 0.0
    @JvmField var kV = 0.00001
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.1
    @JvmField var maxVel = 400.0
    @JvmField var maxAccel = 700.0
    @JvmField var allowedPositionError = 0.0
    @JvmField var disabledPosition = -62.0
}