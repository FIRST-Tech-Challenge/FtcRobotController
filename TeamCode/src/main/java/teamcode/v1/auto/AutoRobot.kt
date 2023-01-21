package teamcode.v1.auto

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.LiftConstants
import teamcode.v1.subsystems.Arm
import teamcode.v1.subsystems.Claw
import teamcode.v1.subsystems.Lift
import teamcode.v1.subsystems.Guide

class AutoRobot(startPose: Pose) {
    private val hardware = AutoHardware(startPose)

    val drive = KMecanumOdoDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
        hardware.odometry,
        true
    )

    val arm = Arm(hardware.armMotor, hardware.limitSwitch)
    val claw = Claw(hardware.clawServo)
    val guide = Guide(hardware.guideServo)
    val lift = Lift(
        hardware.liftLeadMotor,
        hardware.liftSecondMotor
    )

    init {
        arm.setPos(ArmConstants.autoHomePos)
        lift.setPos(LiftConstants.homePos)
    }
}