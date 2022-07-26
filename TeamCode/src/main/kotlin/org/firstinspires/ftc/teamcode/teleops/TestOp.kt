package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.components.deadwheels.Deadwheels
import org.firstinspires.ftc.teamcode.components.deadwheels.initializedDeadwheels
import org.firstinspires.ftc.teamcode.components.deadwheels.logDeadwheelData
import org.firstinspires.ftc.teamcode.components.motors.Motors
import org.firstinspires.ftc.teamcode.components.motors.initializedDriveMotors
import org.firstinspires.ftc.teamcode.util.initializableOnce
import kotlin.math.abs
import kotlin.math.absoluteValue

@TeleOp(name = "TestOpKt")
class TestOp : OpMode() {
    private var motors: Motors by initializableOnce()
    private var deadwheels: Deadwheels by initializableOnce()

    override fun init() {
        motors = initializedDriveMotors(hardwareMap)
        deadwheels = initializedDeadwheels(motors)
    }

    override fun loop() {
        drive()
        deadwheels.logDeadwheelData(telemetry) { it.ticks }
        updateTelemetry(telemetry)
    }

    private fun drive() = with(gamepad1) {
        val triggered = abs(left_stick_y) > 0.1 || abs(left_stick_x) > 0.1 || abs(right_stick_x) > 0.1

        var flp = left_stick_y - left_stick_x - right_stick_x
        var frp = -left_stick_y - left_stick_x - right_stick_x
        var blp = left_stick_y + left_stick_x - right_stick_x
        var brp = -left_stick_y + left_stick_x - right_stick_x

        if (triggered) {
            val max = listOf(flp, frp, blp, brp).maxByOrNull(Float::absoluteValue)!!
            if (max > 1) {
                flp /= max
                frp /= max
                blp /= max
                brp /= max
            }
        }

        val powerMulti = if (triggered) {
            if (right_trigger > 0.5) 0.35 else 1.0
        } else 0.0

        motors.frontLeft.power = flp * powerMulti
        motors.frontRight.power = frp * powerMulti
        motors.backLeft.power = blp * powerMulti
        motors.backRight.power = brp * powerMulti
    }
}