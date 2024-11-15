package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.helpers.bulkCachingMode
import org.firstinspires.ftc.teamcode.intothedeep.Dave

@TeleOp(group = "Into The Deep", name = "Tuning")
class TuningOp : OpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun init() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()
    }

    override fun loop() {
        dave.velocity = gamepad1.povVelocity()

        armControl(gamepad2)

        dave.update()
    }

    fun armControl(gamepad: Gamepad) {
        if (gamepad.dpad_up) dave.armUp()
        if (gamepad.dpad_down) dave.armDown()

        if (gamepad.dpad_left) dave.extend()
        if (gamepad.dpad_right) dave.retract()

        if (gamepad.right_trigger > 0) dave.closePincer()
        if (gamepad.right_bumper) dave.openPincer()

        if (gamepad.left_trigger > 0) dave.retractFullPower()
        if (gamepad.left_bumper) dave.resetExtension()
    }
}

fun Gamepad.povVelocity() = PoseVelocity2d(
    Vector2d(-this.left_stick_y.toDouble(), this.left_stick_x.toDouble()),
    this.right_stick_x.toDouble()
)