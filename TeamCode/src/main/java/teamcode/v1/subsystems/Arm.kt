package teamcode.v1.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val motor: KMotor, val switch : KLimitSwitch) : Subsystem() {

    fun setPos(pos: Double) {
        motor.setProfileTarget(pos)
    }
}