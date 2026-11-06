package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystem.TaskManager

@Configurable
@TeleOp
class TeleOp : FullOpMode() {
    override fun init() {
    }

    override fun init_loop() {
        super.init_loop()
    }

    override fun start() {
    }

    override fun loop() {
        super.loop()
        controls()
    }

    fun controls() {
        follower.manual(
            -(gamepad1.left_stick_y).toDouble(),
            (gamepad1.left_trigger - gamepad1.right_trigger).toDouble(),
            -(gamepad1.left_stick_x).toDouble()
        )
        telemetry.fullTelemtry()
    }
    fun Telemetry.fullTelemtry() {
        taskManager.printState(this)
    }
}
