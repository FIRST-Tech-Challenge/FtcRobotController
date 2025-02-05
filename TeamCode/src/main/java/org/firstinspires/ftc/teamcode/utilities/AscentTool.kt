package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Hardware

@TeleOp(name = "util: Lv. 2 Ascent", group = "Utilities")
class AscentTool : LinearOpMode() {
    val tele = telemetry

    override fun runOpMode() {
        val hardware = Hardware(hardwareMap)
        hardware.sharedHardwareInit()

        waitForStart()

        while (opModeIsActive()) {
            hardware.rightAscent.power =
                if (gamepad1.y) 0.5 // up (towards vertical)
                else if (gamepad1.a) -0.5 // down (towards folded)
                else 0.0
            hardware.leftAscent.power =
                if (gamepad1.dpad_up) 0.5 // down (towards folded)
                else if (gamepad1.dpad_down) -0.5 // up (towards vertical)
                else 0.0

            tele.addData("left ascent encoder", hardware.leftAscentEnc.getCurrentPosition())
            tele.addData("right ascent encoder", hardware.rightAscentEnc.getCurrentPosition())
            tele.update()
        }
    }
}