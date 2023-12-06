package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.BotShared

@TeleOp(name = "Full Manual Drive Controls", group = "Test")
class FullManual : OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
    }

    override fun loop() {

        val fl = shared.motorLeftFront
        val fr = shared.motorRightFront
        val bl = shared.motorLeftBack
        val br = shared.motorRightBack

        val powers = object {
            var fl = gamepad1.left_stick_x.toDouble()
            var fr = gamepad1.right_stick_x.toDouble()
            var bl = -gamepad1.left_stick_y.toDouble()
            var br = -gamepad1.right_stick_y.toDouble()
        }

        fl.power = powers.fl
        fr.power = powers.fr
        bl.power = powers.bl
        br.power = powers.br

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
//        telemetry.addLine("Yaw Difference (bot - input): " + )

        telemetry.addLine("Left Stick X: " + gamepad1.left_stick_x)
        telemetry.addLine("Left Stick Y: " + -gamepad1.left_stick_y)
        telemetry.update()

        shared.update()
    }
}