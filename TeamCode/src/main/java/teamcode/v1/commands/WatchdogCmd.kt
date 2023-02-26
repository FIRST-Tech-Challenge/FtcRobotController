package teamcode.v1.commands

import com.asiankoala.koawalib.command.commands.Cmd

class WatchdogCmd(
    private val toSchedule: Cmd,
    private val condition: () -> Boolean,
) : Cmd() {
    override fun execute() {
        if (condition.invoke()) {
            toSchedule.schedule()
        }
    }

    override val isFinished: Boolean = false
}