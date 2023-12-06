package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.BotShared

@TeleOp(name = "Unwind Truss")
class UnwindTruss : OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
    }

    override fun start() {
        super.start()
        shared.motorTruss?.power = gamepad1.right_trigger.toDouble() - gamepad1.right_trigger.toDouble()
    }

    override fun loop() {
        shared.update()
    }
}