package teamcode.mule

import teamcode.mule.subsystems.MuleArm
import teamcode.mule.MuleHardware

class MuleRobot() {
    val hardware = MuleHardware()

    val arm = MuleArm(hardware.armMotor)

    init {
        arm.setPos(-67.7)
    }
}