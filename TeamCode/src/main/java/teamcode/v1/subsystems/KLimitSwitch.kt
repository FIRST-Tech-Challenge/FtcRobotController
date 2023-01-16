package teamcode.v1.subsystems

import com.asiankoala.koawalib.hardware.KDevice
import com.qualcomm.robotcore.hardware.DigitalChannel

@Suppress("unused")
class KLimitSwitch(name: String) : KDevice<DigitalChannel>(name), KBoolean {

    override fun invokeBoolean(): Boolean {
        return device.state
    }
}