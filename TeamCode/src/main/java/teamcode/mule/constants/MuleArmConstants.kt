package teamcode.mule.constants

import com.acmerobotics.dashboard.config.Config

@Config
object MuleArmConstants {
    @JvmField var ticksPerUnit = 184.0/90.0
    @JvmField var homePos = -69.0
    @JvmField var highPos = 130.0
    @JvmField var homeWaitPos = -20.0
    @JvmField var kP = 0.02
    @JvmField var kI = 0.0
    @JvmField var kD = 0.002
    @JvmField var kS = 0.03
    @JvmField var kV = 0.00001
    @JvmField var kA = 0.0
    @JvmField var kCos = 0.25
    @JvmField var maxVel = 400.0
    @JvmField var maxAccel = 700.0
    @JvmField var allowedPositionError = 1.0
}