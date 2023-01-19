package teamcode.v1.constants

import com.acmerobotics.dashboard.config.Config

@Config
object ClawConstants {
    @JvmField var closePos = 0.65
    @JvmField var openPos = 0.4
    @JvmField var outtakePos = 0.55
    @JvmField var intakePos = 0.4
    @JvmField var distanceThreshold = 0.0
}
