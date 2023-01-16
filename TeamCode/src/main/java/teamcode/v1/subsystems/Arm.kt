package teamcode.v1.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val motor: KMotor, val switch : KLimitSwitch) : Subsystem() {

    val detect get() = switch.invokeBoolean()

    fun setPos(pos: Double) {
        motor.setProfileTarget(pos)
    }
}