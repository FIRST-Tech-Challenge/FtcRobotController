package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.helpers.PowerVector
import org.firstinspires.ftc.teamcode.helpers.mecanumDrive

@TeleOp(name = "Egr TeleOp", group = "")
class EgrTeleOp : OpMode() {
    /* This TeleOp is designed to be used with the `sally` configuration.
     *
     *
     */

    val omniDrive by lazy { this.hardwareMap.mecanumDrive() }

    override fun init() {
        omniDrive.zeroEncoders()
        omniDrive.telemetry = this.telemetry

    }

    override fun loop() {
        omniDrive.powerVector = PowerVector(
            axial = -this.gamepad1.left_stick_y.toDouble(),
            lateral = this.gamepad1.left_stick_x.toDouble(),
            yaw = this.gamepad1.right_stick_x.toDouble()
        )
    }
}