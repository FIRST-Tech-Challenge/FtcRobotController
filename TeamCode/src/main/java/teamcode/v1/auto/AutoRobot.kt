package teamcode.v1.auto

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import teamcode.v1.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

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

    val arm = Arm(hardware.armMotor)
    val claw = Claw(hardware.clawServo)
    val lift = Lift(
        hardware.liftLeadMotor,
        hardware.liftSecondMotor
    )

    init {
        arm.setPos(210.0)
        lift.setPos(0.0)
    }
}