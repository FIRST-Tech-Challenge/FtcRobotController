package teamcode.v1.constants

import com.acmerobotics.dashboard.config.Config

@Config
object LiftConstants {
    @JvmField var ticksPerUnit = 1950.0/18.0
    @JvmField var homePos = 0.0
    @JvmField var groundPos = 0.0
    @JvmField var lowPos = 0.0
    @JvmField var midPos = 5.0
    @JvmField var highPos = 15.5
    @JvmField var kP = 0.65
    @JvmField var kI = 0.0
    @JvmField var kD = 0.005
    @JvmField var kS = 0.02
    @JvmField var kV = 0.007
    @JvmField var kA = 0.0
    @JvmField var kG = 0.02
    @JvmField var maxVel = 600.0
    @JvmField var maxAccel = 300.0
    @JvmField var disabledPosition = 0.0
    @JvmField var allowedPositionError = 2.0
}