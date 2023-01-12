package teamcode.v1

import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import teamcode.v1.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class Robot() {
    val hardware = Hardware()

    val drive = KMecanumDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
    )

    val arm = Arm(hardware.armMotor)
    val claw = Claw(hardware.clawServo)
    val lift = Lift(hardware.liftLeadMotor, hardware.liftSecondMotor)

    init {
        arm.setPos(-71.0)
        lift.setPos(0.0)
    }
}