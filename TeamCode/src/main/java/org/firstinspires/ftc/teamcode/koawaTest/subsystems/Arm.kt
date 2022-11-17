package asiankoala.testing.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val motor: KMotor) : Subsystem() {
    companion object {
        const val topPosition = 35.0
        const val midPosition = -15.0
        const val bottomPosition = -30.0
        const val sharedPosition = -30.0
        const val sharedLow = 25.0
        const val sharedHigh = 35.0
        const val homePosition = -35.0
        const val armIntakeExt = -32.0
        const val armIntakePos = -40.0
        const val initPos = -50.0
    }
}