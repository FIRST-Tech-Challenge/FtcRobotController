package teamcode.v1.auto

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.LiftConstants
import teamcode.v1.subsystems.*
import teamcode.v1.vision.Vision

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
    val whacker = Whacker(hardware.whackerServo)
    val vision = Vision()
    val lift = Lift(
        hardware.liftLeadMotor,
        hardware.liftSecondMotor
    )

    init {
        arm.setPos(ArmConstants.autoHomePos)
        lift.setPos(LiftConstants.homePos)
    }
}