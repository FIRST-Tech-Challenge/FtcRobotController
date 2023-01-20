package teamcode.mule

import teamcode.v1.subsystems.Claw
import teamcode.mule.subsystems.MuleArm

class MuleRobot() {
    val hardware = MuleHardware()

    val arm = MuleArm(hardware.armMotor)
    val claw = Claw(hardware.clawServo)

    init {
        arm.setPos(-67.7)
    }
}