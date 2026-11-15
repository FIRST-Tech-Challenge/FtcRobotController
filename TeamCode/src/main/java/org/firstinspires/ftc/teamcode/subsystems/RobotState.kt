package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore
import kotlin.math.hypot
import kotlin.time.Duration.Companion.seconds

class RobotState(private val hw: GerthaHardwareMap, val isRed: Boolean = true) {

    val initPose = Pose2D (DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
    var ppX: Double = 0.0
    var ppY: Double = 0.0
    var ppThetaDeg: Double = 0.0
    var ppThetaRad: Double = 0.0

    var velXM: Double = 0.0
    var velYM: Double = 0.0

    var velXInch: Double = 0.0
    var velYInch: Double = 0.0
    var velInch: Double = 0.0

    var angVelRad: Double = 0.0

    fun update() {
        hw.pinpoint.update()

        val ppPose = hw.pinpoint.position
        ppX = ppPose.getX(DistanceUnit.INCH)
        ppY = ppPose.getY(DistanceUnit.INCH)
        ppThetaRad = ppPose.getHeading(AngleUnit.RADIANS)
        ppThetaDeg = ppPose.getHeading(AngleUnit.DEGREES)

        velXM = hw.pinpoint.getVelX(DistanceUnit.METER)
        velYM = hw.pinpoint.getVelY(DistanceUnit.METER)

        velXInch = hw.pinpoint.getVelX(DistanceUnit.INCH)
        velYInch = hw.pinpoint.getVelY(DistanceUnit.INCH)
        velInch = hypot(velXInch, velYInch)

        angVelRad = hw.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)

    }

}