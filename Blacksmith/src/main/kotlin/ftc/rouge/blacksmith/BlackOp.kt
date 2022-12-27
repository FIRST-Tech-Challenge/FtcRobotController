package ftc.rouge.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rouge.blacksmith.messenger.Messenger

abstract class BlackOp : LinearOpMode() {
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    @JvmField
    protected var hwMap = hardwareMap

    abstract fun run()

    final override fun runOpMode() {
        hwMap = hardwareMap
        Messenger.emit(STARTING_MSG)
        run()
    }

    companion object {
        var STARTING_MSG = 3248023743480398723L
    }
}
