package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.components.deadwheels.Deadwheels
import org.firstinspires.ftc.teamcode.components.deadwheels.initializedDeadwheels
import org.firstinspires.ftc.teamcode.components.deadwheels.logDeadwheelData
import org.firstinspires.ftc.teamcode.components.deadwheels.snapshotTicks
import org.firstinspires.ftc.teamcode.components.motors.Motors
import org.firstinspires.ftc.teamcode.components.motors.initializedMotors
import org.firstinspires.ftc.teamcode.components.motors.logShooterData
import org.firstinspires.ftc.teamcode.components.servos.Servos
import org.firstinspires.ftc.teamcode.components.servos.initializedServos
import org.firstinspires.ftc.teamcode.components.servos.logServoData
import org.firstinspires.ftc.teamcode.util.initializableOnce
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.log

const val INDEXER_BACK = .53
const val INDEXER_FORWARD = .58

@TeleOp(name = "TestOpKt")
class TestOp : OpMode() {
    private var motors: Motors by initializableOnce()
    private var deadwheels: Deadwheels by initializableOnce()

    private var servos: Servos by initializableOnce()

    private var elapsedTime = ElapsedTime()

    override fun init() {
        motors = initializedMotors(hardwareMap)
        deadwheels = initializedDeadwheels(motors)
        servos = initializedServos(hardwareMap)

        servos.indexer.position = INDEXER_BACK
    }

    override fun start() {
        elapsedTime.reset()
    }

    override fun loop() {
        drive()
        shoot()
-        motors.logShooterData(telemetry) { it.power }
        servos.logServoData(telemetry) { it.position }

        updateTelemetry(telemetry)
    }

    private fun drive() = with(gamepad1) {
        val triggered =
            abs(left_stick_y) > 0.1 || abs(left_stick_x) > 0.1 || abs(right_stick_x) > 0.1

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
            if (left_trigger > 0.5) 0.35 else 1.0
        } else 0.0

        motors.frontLeft.power = flp * powerMulti
        motors.frontRight.power = frp * powerMulti
        motors.backLeft.power = blp * powerMulti
        motors.backRight.power = brp * powerMulti
    }

    private fun shoot(toggled: Boolean = false) {
        motors.shooter.power = if (gamepad1.right_trigger > .6f)
            gamepad1.right_trigger.toDouble()
        else 0.0
        servos.indexer.position = if (gamepad1.a || toggled) INDEXER_FORWARD + .02 else INDEXER_BACK - .02
    }
}