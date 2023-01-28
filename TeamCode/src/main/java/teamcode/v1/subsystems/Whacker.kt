package teamcode.v1.subsystems

import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Whacker(private val servo : KServo) : Subsystem() {
    fun setPos(pos: Double) {
        servo.position = pos
    }
}