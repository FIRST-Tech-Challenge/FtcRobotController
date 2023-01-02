package teamcode.mule

import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import teamcode.mule.subsystems.MuleArm
import teamcode.mule.MuleHardware

class MuleRobot() {
    val hardware = MuleHardware()

    val arm = MuleArm(hardware.armMotor)
    val claw = Claw(hardware.clawServo)

    init {
        arm.setPos(-67.7)
    }
}