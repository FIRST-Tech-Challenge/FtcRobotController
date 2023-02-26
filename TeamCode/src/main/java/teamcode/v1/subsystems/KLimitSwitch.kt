package teamcode.v1.subsystems

import com.asiankoala.koawalib.command.commands.Cmd
import teamcode.v1.commands.WatchdogCmd
import com.asiankoala.koawalib.hardware.KDevice
import com.qualcomm.robotcore.hardware.DigitalChannel

class KLimitSwitch(name: String) : KDevice<DigitalChannel>(name), () -> Boolean {
    override fun invoke() = device.state
    fun onPress(cmd: Cmd) {
        + WatchdogCmd(cmd, ::invoke)
    }
}