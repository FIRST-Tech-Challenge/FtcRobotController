package teamcode.v1.subsystems

import com.asiankoala.koawalib.hardware.sensor.KDistanceSensor
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Guide(private val servo : KServo, private val sensorMain: KDistanceSensor, private val sensorTwo: KDistanceSensor ) : Subsystem() {
    private var isReading = false
    val lastRead get() = sensorMain.lastRead
    val lastReadTwo get() = sensorTwo.lastRead

    fun startReading() {
        isReading = true
    }

    fun stopReading() {
        isReading = false
    }

    fun setPos(pos: Double) {
        servo.position = pos
    }

    override fun periodic() {
        if(isReading) {
            sensorMain.periodic()
            sensorTwo.periodic()
        }
    }
}