package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.components.deadwheels.Deadwheels
import org.firstinspires.ftc.teamcode.components.deadwheels.initializedDeadwheels
import org.firstinspires.ftc.teamcode.components.motors.DriveMotors
import org.firstinspires.ftc.teamcode.components.motors.initializedDriveMotors
import org.firstinspires.ftc.teamcode.components.shooter.Shooter
import org.firstinspires.ftc.teamcode.components.shooter.initializedShooter
import org.firstinspires.ftc.teamcode.util.LateInitVal
import kotlin.math.abs
import kotlin.math.absoluteValue

@TeleOp(name = "TestOpKt")
class TestOp : OpMode() {
    private var motors: DriveMotors by LateInitVal()
    private var deadwheels: Deadwheels by LateInitVal()
    private var shooter: Shooter by LateInitVal()

    //TODO: Probably remove the deadwheels from the TeleOp
    override fun init() {
        motors = initializedDriveMotors(hardwareMap)
        deadwheels = initializedDeadwheels(motors)
        shooter = initializedShooter(hardwareMap)
    }

    override fun loop() {
        drive()
        shoot()

        deadwheels.snapshotTicks()
        updateTelemetry(telemetry)
    }

    private fun drive() = with(gamepad1) {
        val triggered =
            abs(left_stick_y) > 0.1 || abs(left_stick_x) > 0.1 || abs(left_trigger) > 0.1

        var flp = left_stick_y - left_stick_x - right_stick_x
        var frp = -left_stick_y - left_stick_x - right_stick_x
        var blp = left_stick_y + left_stick_x - right_stick_x
        var brp = -left_stick_y + left_stick_x - right_stick_x

        val max = listOf(flp, frp, blp, brp).maxByOrNull(Float::absoluteValue)!!
        if (max > 1) {
            flp /= max
            frp /= max
            blp /= max
            brp /= max
        }

        val powerMulti = when {
            !triggered -> 0.0
            left_trigger > 0.5 -> 0.35
            else -> 1.0
        }

        motors.frontLeft.power = flp * powerMulti
        motors.frontRight.power = frp * powerMulti
        motors.backLeft.power = blp * powerMulti
        motors.backRight.power = brp * powerMulti
    }

    private fun shoot() = with(shooter) {
        motor.power = gamepad1.right_trigger.toDouble().takeIf { it > 0.6 } ?: 0.0
        setIndexerToggled(gamepad1.a)
    }
}