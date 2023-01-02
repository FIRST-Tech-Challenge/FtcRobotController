package teamcode.mule.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class MuleArm(val motor: KMotor) : Subsystem() {
    fun setPos(pos: Double) {
        motor.setProfileTarget(pos)
    }
}