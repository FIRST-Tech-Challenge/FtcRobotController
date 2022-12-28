@file:Suppress("HasPlatformType")

package ftc.rouge.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class BlackOp : LinearOpMode() {
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    @JvmField
    protected var hwMap = hardwareMap

    abstract fun go()

    final override fun runOpMode() {
        hwMap = hardwareMap
        Scheduler.emit(STARTING_MSG)
        go()
    }

    companion object {
        var STARTING_MSG = 3248023743480398723L
    }
}
